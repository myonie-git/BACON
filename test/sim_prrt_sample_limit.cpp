#include <ompl/base/PlannerData.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSampler.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/util/RandomNumbers.h>

#include <collision_env.h>
#include <collision_object.h>
#include <urdf_model.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <getopt.h>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace fs = std::filesystem;
namespace ob = ompl::base;
namespace og = ompl::geometric;

#ifndef CDU_CIMPL_SOURCE_DIR
#define CDU_CIMPL_SOURCE_DIR ""
#endif

struct RobotSpec
{
    std::string id;
    fs::path urdf;
    std::string rootLink;
};

static unsigned int defaultThreadCount()
{
    unsigned int hw = std::thread::hardware_concurrency();
    if (hw == 0)
        hw = 1;
    return std::min(4u, hw);
}

static void printVector(std::ostream &out, const std::vector<double> &v)
{
    out << "[";
    for (std::size_t i = 0; i < v.size(); ++i)
    {
        out << v[i];
        if (i + 1 < v.size())
            out << ", ";
    }
    out << "]";
}

static void printUsage(std::ostream &out, const char *argv0)
{
    out << "用法: " << (argv0 ? argv0 : "sim_prrt_sample_limit") << " [options]\n\n"
        << "常用参数:\n"
        << "  --robot <id>            fanuc/go1/jaco2/jaco3/panda/prbt (默认: panda)\n"
        << "  --env-dir <dir>         环境目录(默认: <repo>/env/48)\n"
        << "  --env-id <n>            环境文件编号(默认: 1 => <env-dir>/1)\n"
        << "  --env-file <path>       直接指定环境文件(优先级最高)\n"
        << "  --print-joints          打印该 robot 的关节顺序(规划维度)并退出\n"
        << "\n规划参数:\n"
        << "  --threads <n>           pRRT 线程数(默认: min(4, CPU核数))\n"
        << "  --range <r>             扩展步长(0=>自动, 默认: 0)\n"
        << "  --goal-bias <p>         采样到 goal 的概率(默认: 0.05)\n"
        << "  --samples <n>           固定采样次数N，达到后立即终止(默认: 50000)\n"
        << "  --joint-low <v>         所有维度下界(默认: -1.57)\n"
        << "  --joint-high <v>        所有维度上界(默认: 1.57)\n"
        << "  --goal-tol <v>          目标容差(默认: 1e-3)\n"
        << "  --start \"v0 v1 ...\"     起点关节向量(空格或逗号分隔；包含空格时请加引号)\n"
        << "  --goal  \"v0 v1 ...\"     终点关节向量(空格或逗号分隔；包含空格时请加引号)\n"
        << "  --seed <u32>            OMPL 全局随机种子(可复现)\n"
        << "\n路径定位:\n"
        << "  --repo <path>           cdu_cimpl 工程根目录(默认: CDU_CIMPL_ROOT 或编译期路径)\n"
        << "\n其它:\n"
        << "  -h, --help              打印帮助\n";
}

static std::vector<double> parseDoublesList(const std::string &s)
{
    // 允许空格/逗号混合分隔，例如: \"0 0, 1.2, -3\"
    std::vector<double> out;
    std::stringstream ss(s);
    std::string chunk;
    while (std::getline(ss, chunk, ','))
    {
        std::stringstream cs(chunk);
        double v = 0.0;
        while (cs >> v)
            out.push_back(v);
    }
    return out;
}

static bool sampleValidState(const ob::SpaceInformationPtr &si, ob::State *out, unsigned int maxAttempts)
{
    if (!si || out == nullptr || maxAttempts == 0)
        return false;

    const ob::StateSamplerPtr sampler = si->allocStateSampler();
    for (unsigned int k = 0; k < maxAttempts; ++k)
    {
        sampler->sampleUniform(out);
        if (si->isValid(out))
            return true;
    }
    return false;
}

static fs::path resolveRepoRoot(const std::string &repoOpt)
{
    if (!repoOpt.empty())
        return fs::path(repoOpt);

    if (const char *env = std::getenv("CDU_CIMPL_ROOT"); env && *env)
        return fs::path(env);

    if (std::string(CDU_CIMPL_SOURCE_DIR).size() > 0)
        return fs::path(CDU_CIMPL_SOURCE_DIR);

    return fs::current_path();
}

static RobotSpec getRobotSpec(const std::string &robotId, const fs::path &repoRoot)
{
    const auto mk = [&](std::string id, fs::path urdfRel, std::string root) {
        RobotSpec spec;
        spec.id = std::move(id);
        spec.urdf = repoRoot / std::move(urdfRel);
        spec.rootLink = std::move(root);
        return spec;
    };

    if (robotId == "fanuc")
        return mk("fanuc", "fanuc_description.urdf", "base_link");
    if (robotId == "go1")
        return mk("go1", "go1.urdf", "base");
    if (robotId == "jaco2")
        return mk("jaco2", "j2n6s200_standalone.urdf", "root");
    if (robotId == "jaco3")
        return mk("jaco3", "j2n6s300_standalone.urdf", "root");
    if (robotId == "panda")
        return mk("panda", "panda_description.urdf", "panda_link0");
    if (robotId == "prbt")
        return mk("prbt", "prbt_description.urdf", "prbt_base_link");

    throw std::runtime_error("未知 robot: " + robotId + " (支持: fanuc/go1/jaco2/jaco3/panda/prbt)");
}

static bool isActiveJointType(const std::string &type)
{
    return type == "revolute" || type == "continuous" || type == "prismatic";
}

static std::vector<std::string> extractActiveJointNamesInDfsOrder(URDFModel &model, const std::string &rootLink)
{
    model.assignJointSerialNumbers(rootLink);

    std::vector<const Joint *> active;
    active.reserve(model.joints.size());
    for (auto &j : model.joints)
    {
        if (j.serial >= 0 && isActiveJointType(j.type))
            active.push_back(&j);
    }

    std::sort(active.begin(), active.end(), [](const Joint *a, const Joint *b) {
        if (a->serial != b->serial)
            return a->serial < b->serial;
        return a->name < b->name;
    });

    std::vector<std::string> names;
    names.reserve(active.size());
    for (const Joint *j : active)
        names.push_back(j->name);

    return names;
}

class ThreadSafeEnvCollisionChecker
{
public:
    using S = double;
    using Clock = std::chrono::steady_clock;

    // 全局计数：记录状态有效性检查调用次数（线程安全）
    static std::atomic<std::uint64_t> callCount;
    // 总耗时(纳秒)：记录 isValid 执行耗时
    static std::atomic<std::uint64_t> validNs;
    // 墙钟累积：记录至少有一个 isValid 在执行的墙钟时间（纳秒）
    static std::atomic<std::uint64_t> validWallNs;
    static std::atomic<std::int64_t> validWallStartNs;
    static std::atomic<int> activeValid;

    static void resetStats()
    {
        callCount.store(0, std::memory_order_relaxed);
        validNs.store(0, std::memory_order_relaxed);
        validWallNs.store(0, std::memory_order_relaxed);
        validWallStartNs.store(0, std::memory_order_relaxed);
        activeValid.store(0, std::memory_order_relaxed);
    }

    ThreadSafeEnvCollisionChecker(std::string urdfPath, std::string rootLink, std::vector<std::string> jointNames,
                                  const CollisionEnv<S> *env, const URDFModel &modelTemplate)
      : baseModel_(cloneModel(modelTemplate)),
        urdfPath_(std::move(urdfPath)),
        rootLink_(std::move(rootLink)),
        jointNames_(std::move(jointNames)),
        env_(env)
    {
        if (env_ == nullptr)
            throw std::runtime_error("env 指针为空");
        if (jointNames_.empty())
            throw std::runtime_error("关节列表为空");
        if (baseModel_.links.empty() || baseModel_.joints.empty())
            throw std::runtime_error("预加载 URDF 为空: " + urdfPath_);
    }

    std::size_t dof() const
    {
        return jointNames_.size();
    }

    bool isValid(const double *q, std::size_t n) const
    {
        ThreadContext &ctx = getThreadContext();
        if (!ctx.initialized)
            initThreadContext(ctx);

        const auto t0 = Clock::now();
        const auto t0Ns =
            static_cast<std::int64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(t0.time_since_epoch()).count());
        const int prevActive = activeValid.fetch_add(1, std::memory_order_acq_rel);
        if (prevActive == 0)
            validWallStartNs.store(t0Ns, std::memory_order_relaxed);

        callCount.fetch_add(1, std::memory_order_relaxed);
        if (q == nullptr || n != jointNames_.size())
        {
            const auto t1 = Clock::now();
            const auto dur = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
            validNs.fetch_add(static_cast<std::uint64_t>(dur), std::memory_order_relaxed);

            const int after = activeValid.fetch_sub(1, std::memory_order_acq_rel) - 1;
            if (after == 0)
            {
                const auto t1Ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t1.time_since_epoch()).count();
                validWallNs.fetch_add(static_cast<std::uint64_t>(t1Ns - validWallStartNs.load(std::memory_order_relaxed)),
                                      std::memory_order_relaxed);
            }
            return false;
        }

        // 更新关节角
        for (std::size_t i = 0; i < n; ++i)
            ctx.model.jointAngles[jointNames_[i]] = q[i];

        // 正向运动学：更新每个 link 的世界位姿
        ctx.model.calculateWorldCoordinates(rootLink_);

        // 构造机器人各碰撞几何对应的 CollisionObject，然后与环境做碰撞
        ctx.robotCollisionObjects.clear();
        ctx.robotCollisionObjects.reserve(ctx.model.collisionGeometries.size());
        for (const auto &geomPtr : ctx.model.collisionGeometries)
            ctx.robotCollisionObjects.emplace_back(*geomPtr);

        
        for (auto &obj : ctx.robotCollisionObjects)
        {
            if (env_->collide(&obj, nullptr))
            {
                const auto t1 = Clock::now();
                const auto dur = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
                validNs.fetch_add(static_cast<std::uint64_t>(dur), std::memory_order_relaxed);

                const int after = activeValid.fetch_sub(1, std::memory_order_acq_rel) - 1;
                if (after == 0)
                {
                    const auto t1Ns =
                        std::chrono::duration_cast<std::chrono::nanoseconds>(t1.time_since_epoch()).count();
                    validWallNs.fetch_add(
                        static_cast<std::uint64_t>(t1Ns - validWallStartNs.load(std::memory_order_relaxed)),
                        std::memory_order_relaxed);
                }
                return false;
            }
        }

        const auto t1 = Clock::now();
        const auto dur = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
        validNs.fetch_add(static_cast<std::uint64_t>(dur), std::memory_order_relaxed);

        const int after = activeValid.fetch_sub(1, std::memory_order_acq_rel) - 1;
        if (after == 0)
        {
            const auto t1Ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t1.time_since_epoch()).count();
            validWallNs.fetch_add(static_cast<std::uint64_t>(t1Ns - validWallStartNs.load(std::memory_order_relaxed)),
                                  std::memory_order_relaxed);
        }
        return true;
    }

private:
    struct ThreadContext
    {
        bool initialized{false};
        URDFModel model;
        std::vector<CollisionObject<S>> robotCollisionObjects;
    };

    ThreadContext &getThreadContext() const
    {
        // 注意：这个 thread_local 是“按线程”共享的；本程序只构造一个 checker，因此足够。
        static thread_local ThreadContext ctx;
        return ctx;
    }

    void initThreadContext(ThreadContext &ctx) const
    {
        ctx.model = cloneModel(baseModel_);

        // 预填充关节键，避免在第一次 set 时触发额外的 map 分配。
        for (const auto &name : jointNames_)
            ctx.model.jointAngles[name] = 0.0;

        ctx.initialized = true;
    }

    static URDFModel cloneModel(const URDFModel &src)
    {
        URDFModel dst;
        dst.joints = src.joints;
        dst.jointAngles = src.jointAngles;

        // 先拷贝 link 骨架，延迟填充 collisions，确保 parentLink 指针能重新绑定到新 link。
        for (const auto &entry : src.links)
        {
            Link linkCopy = entry.second;
            linkCopy.collisions.clear();
            dst.links.emplace(entry.first, std::move(linkCopy));
        }

        // 逐个 link 复制 collision 几何，并重新绑定 parentLink。
        for (const auto &entry : src.links)
        {
            auto dstLinkIt = dst.links.find(entry.first);
            if (dstLinkIt == dst.links.end())
                continue;

            for (const auto &geomPtr : entry.second.collisions)
            {
                auto geomCopy = std::make_shared<CollisionGeom>(*geomPtr);
                geomCopy->parentLink = &dstLinkIt->second;
                dstLinkIt->second.collisions.push_back(geomCopy);
                dst.collisionGeometries.push_back(geomCopy);
            }
        }

        return dst;
    }

    URDFModel baseModel_;
    std::string urdfPath_;
    std::string rootLink_;
    std::vector<std::string> jointNames_;
    const CollisionEnv<S> *env_;
};

std::atomic<std::uint64_t> ThreadSafeEnvCollisionChecker::callCount{0};
std::atomic<std::uint64_t> ThreadSafeEnvCollisionChecker::validNs{0};
std::atomic<std::uint64_t> ThreadSafeEnvCollisionChecker::validWallNs{0};
std::atomic<std::int64_t> ThreadSafeEnvCollisionChecker::validWallStartNs{0};
std::atomic<int> ThreadSafeEnvCollisionChecker::activeValid{0};

// 采样次数与耗时统计（线程安全）
struct SampleStats
{
    std::atomic<std::uint64_t> samples{0};
    std::atomic<std::uint64_t> sampleNs{0};
};

class CountingSampler : public ob::StateSampler
{
public:
    using Clock = std::chrono::steady_clock;

    CountingSampler(const ob::StateSpace *space, ob::StateSamplerPtr inner, SampleStats *stats)
      : ob::StateSampler(space), inner_(std::move(inner)), stats_(stats)
    {
        if (!inner_)
            throw std::runtime_error("inner sampler 为空");
        if (stats_ == nullptr)
            throw std::runtime_error("stats 指针为空");
    }

    void sampleUniform(ob::State *state) override
    {
        const auto t0 = Clock::now();
        inner_->sampleUniform(state);
        const auto t1 = Clock::now();
        stats_->samples.fetch_add(1, std::memory_order_relaxed);
        stats_->sampleNs.fetch_add(static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count()),
                                   std::memory_order_relaxed);
    }

    void sampleUniformNear(ob::State *state, const ob::State *near, double distance) override
    {
        const auto t0 = Clock::now();
        inner_->sampleUniformNear(state, near, distance);
        const auto t1 = Clock::now();
        stats_->samples.fetch_add(1, std::memory_order_relaxed);
        stats_->sampleNs.fetch_add(static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count()),
                                   std::memory_order_relaxed);
    }

    void sampleGaussian(ob::State *state, const ob::State *mean, double stdDev) override
    {
        const auto t0 = Clock::now();
        inner_->sampleGaussian(state, mean, stdDev);
        const auto t1 = Clock::now();
        stats_->samples.fetch_add(1, std::memory_order_relaxed);
        stats_->sampleNs.fetch_add(static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count()),
                                   std::memory_order_relaxed);
    }

private:
    ob::StateSamplerPtr inner_;
    SampleStats *stats_;
};

int main(int argc, char **argv)
{
    std::string repoOpt;
    std::string robotId = "panda";

    std::string envDirOpt;
    int envId = 2;
    std::string envFileOpt;

    unsigned int threads = defaultThreadCount();
    double range = 0.0;  // 0 => OMPL 自动配置
    double goalBias = 0.05;
    std::uint64_t sampleLimit = 50000;

    double jointLow = -3.14;
    double jointHigh = 3.14;
    double goalTolerance = 1e-5;

    std::vector<double> startVec;
    std::vector<double> goalVec;

    bool printJoints = false;

    std::string startStr;
    std::string goalStr;
    bool showHelp = false;
    bool seedProvided = false;
    std::uint_fast32_t seed = 0;

    enum
    {
        OPT_REPO = 1000,
        OPT_ROBOT,
        OPT_ENV_DIR,
        OPT_ENV_ID,
        OPT_ENV_FILE,
        OPT_PRINT_JOINTS,
        OPT_THREADS,
        OPT_RANGE,
        OPT_GOAL_BIAS,
        OPT_SAMPLES,
        OPT_JOINT_LOW,
        OPT_JOINT_HIGH,
        OPT_GOAL_TOL,
        OPT_START,
        OPT_GOAL,
        OPT_SEED,
    };

    static const option longOpts[] = {
        {"help", no_argument, nullptr, 'h'},
        {"repo", required_argument, nullptr, OPT_REPO},
        {"robot", required_argument, nullptr, OPT_ROBOT},
        {"env-dir", required_argument, nullptr, OPT_ENV_DIR},
        {"env-id", required_argument, nullptr, OPT_ENV_ID},
        {"env-file", required_argument, nullptr, OPT_ENV_FILE},
        {"print-joints", no_argument, nullptr, OPT_PRINT_JOINTS},
        {"threads", required_argument, nullptr, OPT_THREADS},
        {"range", required_argument, nullptr, OPT_RANGE},
        {"goal-bias", required_argument, nullptr, OPT_GOAL_BIAS},
        {"samples", required_argument, nullptr, OPT_SAMPLES},
        {"joint-low", required_argument, nullptr, OPT_JOINT_LOW},
        {"joint-high", required_argument, nullptr, OPT_JOINT_HIGH},
        {"goal-tol", required_argument, nullptr, OPT_GOAL_TOL},
        {"start", required_argument, nullptr, OPT_START},
        {"goal", required_argument, nullptr, OPT_GOAL},
        {"seed", required_argument, nullptr, OPT_SEED},
        {nullptr, 0, nullptr, 0},
    };

    int c = 0;
    while ((c = getopt_long(argc, argv, "h", longOpts, nullptr)) != -1)
    {
        switch (c)
        {
        case 'h':
            showHelp = true;
            break;
        case OPT_REPO:
            repoOpt = optarg;
            break;
        case OPT_ROBOT:
            robotId = optarg;
            break;
        case OPT_ENV_DIR:
            envDirOpt = optarg;
            break;
        case OPT_ENV_ID:
            envId = std::stoi(optarg);
            break;
        case OPT_ENV_FILE:
            envFileOpt = optarg;
            break;
        case OPT_PRINT_JOINTS:
            printJoints = true;
            break;
        case OPT_THREADS:
            threads = static_cast<unsigned int>(std::stoul(optarg));
            break;
        case OPT_RANGE:
            range = std::stod(optarg);
            break;
        case OPT_GOAL_BIAS:
            goalBias = std::stod(optarg);
            break;
        case OPT_SAMPLES:
            sampleLimit = static_cast<std::uint64_t>(std::stoull(optarg));
            break;
        case OPT_JOINT_LOW:
            jointLow = std::stod(optarg);
            break;
        case OPT_JOINT_HIGH:
            jointHigh = std::stod(optarg);
            break;
        case OPT_GOAL_TOL:
            goalTolerance = std::stod(optarg);
            break;
        case OPT_START:
            startStr = optarg;
            break;
        case OPT_GOAL:
            goalStr = optarg;
            break;
        case OPT_SEED:
            seed = static_cast<std::uint_fast32_t>(std::stoul(optarg));
            seedProvided = true;
            break;
        default:
            std::cerr << "未知参数，使用 --help 查看用法\n";
            printUsage(std::cerr, argv[0]);
            return 2;
        }
    }

    if (showHelp)
    {
        printUsage(std::cout, argv[0]);
        return 0;
    }

    if (sampleLimit == 0)
    {
        std::cerr << "采样次数必须大于 0\n";
        return 2;
    }

    if (!startStr.empty())
        startVec = parseDoublesList(startStr);
    if (!goalStr.empty())
        goalVec = parseDoublesList(goalStr);
    if (seedProvided)
        ompl::RNG::setSeed(seed);

    const bool startProvided = !startStr.empty();
    const bool goalProvided = !goalStr.empty();

    if (threads < 1)
        threads = 1;
    threads = std::min(threads, 64u);

    if (!(jointHigh > jointLow))
    {
        std::cerr << "Invalid bounds: --joint-high must be > --joint-low" << std::endl;
        return 2;
    }

    const fs::path repoRoot = resolveRepoRoot(repoOpt);
    const RobotSpec spec = getRobotSpec(robotId, repoRoot);

    if (!fs::exists(spec.urdf))
    {
        std::cerr << "URDF not found: " << spec.urdf << std::endl;
        return 2;
    }

    fs::path envFile;
    if (!envFileOpt.empty())
    {
        envFile = fs::path(envFileOpt);
    }
    else
    {
        fs::path envDir = envDirOpt.empty() ? (repoRoot / "env/48") : fs::path(envDirOpt);
        envFile = envDir / std::to_string(envId);
    }

    if (!fs::exists(envFile))
    {
        std::cerr << "Env file not found: " << envFile << std::endl;
        return 2;
    }

    // 读取一次 URDF，用于确定关节顺序(DFS 序)和维度，同时复用给碰撞检查器避免重复加载
    URDFModel modelProbe;
    if (!modelProbe.loadURDF(spec.urdf.string()))
    {
        std::cerr << "Failed to load URDF for probing: " << spec.urdf << std::endl;
        return 2;
    }
    std::vector<std::string> jointNames = extractActiveJointNamesInDfsOrder(modelProbe, spec.rootLink);

    if (jointNames.empty())
    {
        std::cerr << "No active joints found in URDF: " << spec.urdf << std::endl;
        return 2;
    }

    if (printJoints)
    {
        std::cout << "Robot: " << spec.id << "\nURDF : " << spec.urdf << "\nRoot : " << spec.rootLink
                  << "\nActive joints (index -> name):\n";
        for (std::size_t i = 0; i < jointNames.size(); ++i)
            std::cout << "  [" << i << "] " << jointNames[i] << "\n";
        return 0;
    }

    const std::size_t dof = jointNames.size();

    if (!startVec.empty() && startVec.size() != dof)
    {
        std::cerr << "Invalid --start: expected " << dof << " values, got " << startVec.size() << std::endl;
        return 2;
    }
    if (!goalVec.empty() && goalVec.size() != dof)
    {
        std::cerr << "Invalid --goal: expected " << dof << " values, got " << goalVec.size() << std::endl;
        return 2;
    }
    if (startVec.empty())
        startVec.assign(dof, 0.0);
    if (goalVec.empty())
        goalVec.assign(dof, 1.5);

    // 构建环境 BVH
    using S = double;
    std::vector<std::unique_ptr<CollisionObject<S>>> envObjects;
    std::vector<CollisionObject<S> *> envPtrs;

    {
        std::vector<CollisionObject<S> *> raw = readBoxesFromFile<S>(envFile.string());
        envObjects.reserve(raw.size());
        envPtrs.reserve(raw.size());
        for (CollisionObject<S> *p : raw)
        {
            envPtrs.push_back(p);
            envObjects.emplace_back(p);
        }
    }

    CollisionEnv<S> env;
    env.InitTree(envPtrs);

    std::cout << "OMPL version: " << OMPL_VERSION << "\n";
    std::cout << "Planner: pRRT (threads=" << threads << ", range=" << range << ", goal_bias=" << goalBias << ")\n";
    std::cout << "Robot : " << spec.id << "\nURDF  : " << spec.urdf << "\nRoot  : " << spec.rootLink
              << "\nEnv   : " << envFile << " (boxes=" << envPtrs.size() << ")\n";
    std::cout << "DoF   : " << dof << "\nBounds: [" << jointLow << ", " << jointHigh << "]\n";
    std::cout << "Sample limit N : " << sampleLimit << "\n";
    std::cout << "Start : ";
    printVector(std::cout, startVec);
    std::cout << "\nGoal  : ";
    printVector(std::cout, goalVec);
    std::cout << "\n";

    // 配置关节空间
    auto space = std::make_shared<ob::RealVectorStateSpace>(static_cast<unsigned int>(dof));
    ob::RealVectorBounds bounds(static_cast<unsigned int>(dof));
    for (std::size_t i = 0; i < dof; ++i)
    {
        bounds.setLow(static_cast<unsigned int>(i), jointLow);
        bounds.setHigh(static_cast<unsigned int>(i), jointHigh);
    }
    space->setBounds(bounds);

    // 统计采样耗时的 sampler
    auto stats = std::make_shared<SampleStats>();
    space->setStateSamplerAllocator([stats](const ob::StateSpace *sp) {
        ob::StateSamplerPtr inner = sp->allocDefaultStateSampler();
        return std::make_shared<CountingSampler>(sp, std::move(inner), stats.get());
    });

    og::SimpleSetup ss(space);

    // 线程安全的环境碰撞检查器(每个线程各自 loadURDF + FK + 碰撞)
    ThreadSafeEnvCollisionChecker checker(spec.urdf.string(), spec.rootLink, jointNames, &env, modelProbe);

    ss.setStateValidityChecker([space, &checker](const ob::State *state) {
        if (!space->satisfiesBounds(state))
            return false;
        const auto *rv = state->as<ob::RealVectorStateSpace::StateType>();
        return checker.isValid(rv->values, checker.dof());
    });

    const ob::SpaceInformationPtr si = ss.getSpaceInformation();
    si->setup();

    ob::ScopedState<ob::RealVectorStateSpace> start(space);
    ob::ScopedState<ob::RealVectorStateSpace> goal(space);
    for (std::size_t i = 0; i < dof; ++i)
    {
        start[static_cast<unsigned int>(i)] = startVec[i];
        goal[static_cast<unsigned int>(i)] = goalVec[i];
    }

    // 如果用户未显式提供 start/goal，且默认值无效，则自动采样有效状态
    const unsigned int maxSampleAttempts = 5000;

    bool startOk = si->isValid(start.get());
    bool goalOk = si->isValid(goal.get());

    if (!startOk && !startProvided)
    {
        std::cout << "INFO: default start is invalid, sampling a valid start...\n";
        startOk = sampleValidState(si, start.get(), maxSampleAttempts);
    }
    if (!goalOk && !goalProvided)
    {
        std::cout << "INFO: default goal is invalid, sampling a valid goal...\n";
        goalOk = sampleValidState(si, goal.get(), maxSampleAttempts);
    }

    if (!startOk)
    {
        if (startProvided)
        {
            std::cerr << "ERROR: start state is invalid (collision/out-of-bounds).\n"
                         "  提示: 用 --print-joints 查看维度/顺序，然后修正 --start。\n";
        }
        else
        {
            std::cerr << "ERROR: failed to sample a valid start within " << maxSampleAttempts
                      << " attempts.\n"
                         "  可能该环境下没有无碰撞配置，或采样空间过窄。\n"
                         "  建议: 更换 --env-id/--env-file，或调整 --joint-low/--joint-high。\n";
        }
        return 2;
    }
    if (!goalOk)
    {
        if (goalProvided)
        {
            std::cerr << "ERROR: goal state is invalid (collision/out-of-bounds).\n"
                         "  提示: 用 --print-joints 查看维度/顺序，然后修正 --goal。\n";
        }
        else
        {
            std::cerr << "ERROR: failed to sample a valid goal within " << maxSampleAttempts
                      << " attempts.\n"
                         "  可能该环境下没有无碰撞配置，或采样空间过窄。\n"
                         "  建议: 更换 --env-id/--env-file，或调整 --joint-low/--joint-high。\n";
        }
        return 2;
    }

    {
        std::vector<double> startUsed(dof);
        std::vector<double> goalUsed(dof);
        for (std::size_t i = 0; i < dof; ++i)
        {
            startUsed[i] = start[static_cast<unsigned int>(i)];
            goalUsed[i] = goal[static_cast<unsigned int>(i)];
        }
        std::cout << "Using Start: ";
        printVector(std::cout, startUsed);
        std::cout << "\nUsing Goal : ";
        printVector(std::cout, goalUsed);
        std::cout << "\n";
    }

    ss.setStartAndGoalStates(start, goal, goalTolerance);

    auto planner = std::make_shared<og::pRRT>(si);
    planner->setThreadCount(threads);
    planner->setGoalBias(goalBias);
    if (range > 0.0)
        planner->setRange(range);
    ss.setPlanner(planner);

    ss.setup();

    // 重置统计，确保仅统计 solve 阶段
    ThreadSafeEnvCollisionChecker::resetStats();

    // 采样次数达到 N 即终止；可能因多线程略有超出。
    ob::PlannerTerminationCondition ptc([&stats, sampleLimit]() {
        return stats->samples.load(std::memory_order_relaxed) >= sampleLimit;
    });

    const auto t0 = CountingSampler::Clock::now();
    const ob::PlannerStatus solved = ss.solve(ptc);
    const auto t1 = CountingSampler::Clock::now();

    const std::uint64_t totalNs =
        static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
    const std::uint64_t sampled = stats->samples.load(std::memory_order_relaxed);
    const std::uint64_t validNsSum = ThreadSafeEnvCollisionChecker::validNs.load(std::memory_order_relaxed);          // 线程总和
    const std::uint64_t validNsWall = ThreadSafeEnvCollisionChecker::validWallNs.load(std::memory_order_relaxed);      // 墙钟累积
    const std::uint64_t validCalls = ThreadSafeEnvCollisionChecker::callCount.load(std::memory_order_relaxed);
    const std::uint64_t buildNs = (totalNs > validNsWall) ? (totalNs - validNsWall) : 0;

    const bool hasPath = solved;
    if (hasPath)
    {
        std::cout << "Found solution" << (solved.asString() == std::string("Exact solution") ? "" : " (approx)")
                  << " before reaching sample limit.\n";
    }
    else
    {
        std::cout << "Terminated after reaching sample limit (N=" << sampleLimit << ")，未找到路径或未尝试。\n";
    }

    ob::PlannerData pd(si);
    ss.getPlannerData(pd);

    const double totalMs = static_cast<double>(totalNs) / 1e6;
    const double validMsSum = static_cast<double>(validNsSum) / 1e6;
    const double validMsWall = static_cast<double>(validNsWall) / 1e6;
    const double buildMs = static_cast<double>(buildNs) / 1e6;
    const double avgValidUs = validCalls > 0 ? (static_cast<double>(validNsSum) / validCalls) / 1e3 : 0.0;
    const double wallFrac = totalNs > 0 ? (static_cast<double>(validNsWall) / totalNs) * 100.0 : 0.0;

    std::cout << "---- 统计 ----\n";
    std::cout << "总耗时 (wall): " << totalMs << " ms\n";
    std::cout << "状态有效性检查耗时(线程总和): " << validMsSum << " ms\n";
    std::cout << "状态有效性检查耗时(墙钟累计): " << validMsWall << " ms, 占墙钟约 " << wallFrac << "%\n";
    std::cout << "建树/其余耗时估计: " << buildMs << " ms\n";
    std::cout << "采样次数(目标/实际): " << sampleLimit << " / " << sampled << "\n";
    std::cout << "平均单次有效性检查: " << avgValidUs << " us\n";
    std::cout << "节点数: " << pd.numVertices() << ", 边数: " << pd.numEdges() << "\n";
    if (hasPath)
        std::cout << "路径段数: " << ss.getSolutionPath().getStateCount() << "\n";
    std::cout << "状态有效性检查调用次数: " << validCalls << "\n";
    std::cout << "注意: 统计已在 solve 前重置；线程总和可能大于墙钟，墙钟累计为至少一个 isValid 运行的时间段。\n";

    return 0;
}

