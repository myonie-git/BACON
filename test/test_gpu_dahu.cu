#include <iostream>
#include <cmath>
#include <cuda_runtime.h>

template <typename S>
__device__ bool obbDisjoint_gpu(const S* B, const S* T, const S* a, const S* b) {
    S t_val, s_val;
    const S reps = 1e-6;

    S Bf[9];
    for (int i = 0; i < 9; i++) {
        Bf[i] = fabs(B[i]) + reps;
    }

    for (int i = 0; i < 3; i++) {
        t_val = fabs(T[i]);
        S sum = 0.0;
        for (int j = 0; j < 3; j++) {
            sum += Bf[j * 3 + i] * b[j];
        }
        if (t_val > (a[i] + sum))
            return true;
    }

    for (int i = 0; i < 3; i++) {
        S s = 0.0;
        for (int j = 0; j < 3; j++)
            s += B[i * 3 + j] * T[j];
        t_val = fabs(s);
        S sum = 0.0;
        for (int j = 0; j < 3; j++)
            sum += Bf[i * 3 + j] * a[j];
        if (t_val > (b[i] + sum))
            return true;
    }

    for (int i = 0; i < 3; i++) { // A 的轴
        for (int j = 0; j < 3; j++) { // B 的轴
            S s = T[(i + 2) % 3] * B[j * 3 + (i + 1) % 3] - T[(i + 1) % 3] * B[j * 3 + (i + 2) % 3];
            t_val = fabs(s);
            S threshold = a[(i + 1) % 3] * Bf[j * 3 + (i + 2) % 3] + a[(i + 2) % 3] * Bf[j * 3 + (i + 1) % 3]
                        + b[(j + 1) % 3] * Bf[(j + 2) % 3 * 3 + i] + b[(j + 2) % 3] * Bf[(j + 1) % 3 * 3 + i];
            if (t_val > threshold)
                return true;
        }
    }

    return false;
}

template <typename S>
__global__ void obbDisjointKernel(const S* B, const S* T, const S* a, const S* b, bool* results) {
    int idx = blockDim.x * blockIdx.x + threadIdx.x;
    if (idx < 16) {
        results[idx] = obbDisjoint_gpu<S>(B, T, a, b);
    }
}

int main() {
    const int numThreads = 16;
    const int numBlocks = 1;

    float h_B[9] = {1, 1, 1, 1, 1, 1, 1, 1, 1};
    float h_T[3] = { 1, 1, 1};
    float h_a[3] = { 1, 1, 1};
    float h_b[3] = { 1, 1, 1};

    float* d_B;
    float* d_T;
    float* d_a;
    float* d_b;
    bool* d_results;
    bool h_results[numThreads];

    cudaMalloc((void**)&d_B, 9 * sizeof(float));
    cudaMalloc((void**)&d_T, 3 * sizeof(float));
    cudaMalloc((void**)&d_a, 3 * sizeof(float));
    cudaMalloc((void**)&d_b, 3 * sizeof(float));
    cudaMalloc((void**)&d_results, numThreads * sizeof(bool));

    cudaMemcpy(d_B, h_B, 9 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_T, h_T, 3 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_a, h_a, 3 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_b, h_b, 3 * sizeof(float), cudaMemcpyHostToDevice);

    obbDisjointKernel<float><<<numBlocks, numThreads>>>(d_B, d_T, d_a, d_b, d_results);

    cudaMemcpy(h_results, d_results, numThreads * sizeof(bool), cudaMemcpyDeviceToHost);

    for (int i = 0; i < numThreads; i++) {
        std::cout << "Thread " << i << " result: " << h_results[i] << std::endl;
    }

    cudaFree(d_B);
    cudaFree(d_T);
    cudaFree(d_a);
    cudaFree(d_b);
    cudaFree(d_results);

    return 0;
}