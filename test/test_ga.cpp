#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>
#include <random>
#include <cmath>
#include <unordered_map>

using namespace std;

// 随机数生成器
random_device rd;
mt19937 gen(rd());

// 假设的复杂函数 f(x)，需要根据实际情况替换
double complexFunction(const vector<double>& data, const vector<int>& groups) {
    double fitness = 0.0;
    int maxGroup = *max_element(groups.begin(), groups.end());
    for (int g = 0; g <= maxGroup; ++g) {
        vector<double> groupElements;
        for (size_t i = 0; i < data.size(); ++i) {
            if (groups[i] == g) {
                groupElements.push_back(data[i]);
            }
        }
        if (!groupElements.empty()) {
            double mean = accumulate(groupElements.begin(), groupElements.end(), 0.0) / groupElements.size();
            for (double val : groupElements) {
                fitness -= pow(val - mean, 2);
            }
        }
    }
    return fitness;
}

// 个体结构体
struct Individual {
    vector<int> genes;  // 分组方案
    double fitness;     // 适应度值

    Individual(int n) : genes(n), fitness(0.0) {}
};

// 重新映射组号，使组号从0开始且连续
void normalizeGroups(Individual& individual) {
    unordered_map<int, int> groupMapping;
    int newGroupNum = 0;
    for (size_t i = 0; i < individual.genes.size(); ++i) {
        int group = individual.genes[i];
        if (groupMapping.find(group) == groupMapping.end()) {
            groupMapping[group] = newGroupNum++;
        }
        individual.genes[i] = groupMapping[group];
    }
}

// 初始化种群
void initializePopulation(vector<Individual>& population, int populationSize, int n, int maxGroups) {
    uniform_int_distribution<> disGroupNum(1, maxGroups);
    for (int i = 0; i < populationSize; ++i) {
        int x = disGroupNum(gen);  // 随机的组数，最大不超过 maxGroups
        uniform_int_distribution<> disGroup(0, x - 1);
        Individual individual(n);
        for (int j = 0; j < n; ++j) {
            individual.genes[j] = disGroup(gen);
        }
        normalizeGroups(individual);  // 确保组号连续
        population.push_back(individual);
    }
}

// 计算适应度
void evaluateFitness(vector<Individual>& population, const vector<double>& data) {
    for (auto& individual : population) {
        individual.fitness = complexFunction(data, individual.genes);
    }
}

// 选择操作（轮盘赌选择）
void selection(const vector<Individual>& population, vector<Individual>& matingPool) {
    // 修正适应度为非负值
    double minFitness = min_element(population.begin(), population.end(), [](const Individual& a, const Individual& b) {
        return a.fitness < b.fitness;
    })->fitness;

    double offset = 0.0;
    if (minFitness < 0) {
        offset = -minFitness + 1;
    }

    double totalFitness = 0.0;
    for (const auto& individual : population) {
        totalFitness += individual.fitness + offset;
    }

    uniform_real_distribution<> dis(0.0, totalFitness);
    for (size_t i = 0; i < population.size(); ++i) {
        double randValue = dis(gen);
        double sum = 0.0;
        for (const auto& individual : population) {
            sum += individual.fitness + offset;
            if (sum >= randValue) {
                matingPool.push_back(individual);
                break;
            }
        }
    }
}

void crossover(vector<Individual>& population, int n) {
    uniform_int_distribution<> disPoint(1, n - 1);
    for (size_t i = 0; i + 1 < population.size(); i += 2) {
        int crossoverPoint = disPoint(gen);
        for (int j = crossoverPoint; j < n; ++j) {
            swap(population[i].genes[j], population[i + 1].genes[j]);
        }
        // 交叉后规范化组号
        normalizeGroups(population[i]);
        normalizeGroups(population[i + 1]);
    }
}

// 变异操作
void mutation(vector<Individual>& population, int n, double mutationRate, int maxGroups) {
    uniform_real_distribution<> disProb(0.0, 1.0);
    for (auto& individual : population) {
        if (disProb(gen) < mutationRate) {
            uniform_int_distribution<> disPoint(0, n - 1);
            int mutatePoint = disPoint(gen);
            // 生成新的组号，限制在 0 到 maxGroups - 1
            uniform_int_distribution<> disGroup(0, maxGroups - 1);
            individual.genes[mutatePoint] = disGroup(gen);
            // 变异后规范化组号
            normalizeGroups(individual);
        }
    }
}

int main() {
    // 参数设置
    int n = 50;                        // 元素数量
    int populationSize = 100;          // 种群大小
    int numGenerations = 100;          // 迭代次数
    double mutationRate = 0.05;        // 变异率
    int maxGroups = 50;                // 最大组数，可根据需要调整

    // 数据初始化
    vector<double> data(n);
    uniform_real_distribution<> disData(0.0, 100.0);
    for (int i = 0; i < n; ++i) {
        data[i] = disData(gen);
    }

    // 初始化种群
    vector<Individual> population;
    initializePopulation(population, populationSize, n, maxGroups);

    // 遗传算法主循环
    for (int generation = 0; generation < numGenerations; ++generation) {
        // 计算适应度
        evaluateFitness(population, data);

        // 选择操作
        vector<Individual> matingPool;
        selection(population, matingPool);

        // 交叉操作
        crossover(matingPool, n);

        // 变异操作
        mutation(matingPool, n, mutationRate, maxGroups);

        // 更新种群
        population = matingPool;
    }

    // 最优解
    evaluateFitness(population, data);
    auto bestIndividual = max_element(population.begin(), population.end(), [](const Individual& a, const Individual& b) {
        return a.fitness < b.fitness;
    });

    // 输出最优分组方案
    cout << "最优的分组方案：" << endl;
    for (int gene : bestIndividual->genes) {
        cout << gene << " ";
    }
    cout << endl;
    cout << "对应的适应度值：" << bestIndividual->fitness << endl;

    // 输出每组中的元素编号
    int maxGroup = *max_element(bestIndividual->genes.begin(), bestIndividual->genes.end());
    for (int g = 0; g <= maxGroup; ++g) {
        cout << "组 " << g << " 中的元素编号: ";
        for (size_t i = 0; i < bestIndividual->genes.size(); ++i) {
            if (bestIndividual->genes[i] == g) {
                cout << i << " ";
            }
        }
        cout << endl;
    }

    return 0;
}
