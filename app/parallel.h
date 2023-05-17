#pragma once
#include "args.h"
#ifdef USE_TBB
#    include <tbb/parallel_for.h>
#    include <tbb/parallel_reduce.h>
#    include <tbb/parallel_sort.h>
#    include <tbb/task.h>
#endif

template<typename IndexType, typename Function>
void parallelFor(IndexType start, IndexType end, const Function & func) {
    if (start > end) {
        return;
    }
#ifdef USE_TBB
    tbb::parallel_for(start, end, func);
#elif USE_OPENMP
#    pragma omp parallel for
    for (auto i = start; i < end; ++i) {
        func(i);
    }
#else
    for (auto i = start; i < end; ++i) {
        func(i);
    }
#endif
}