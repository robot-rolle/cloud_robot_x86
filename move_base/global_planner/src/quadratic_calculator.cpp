/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <global_planner/quadratic_calculator.h>

namespace global_planner {
float QuadraticCalculator::calculatePotential(float* potential, unsigned char cost, int n, float prev_potential) {
    // 获得邻点的potential
    float u, d, l, r;
    l = potential[n - 1];
    r = potential[n + 1];
    u = potential[n - nx_];
    d = potential[n + nx_];
    //  ROS_INFO("[Update] c: %f  l: %f  r: %f  u: %f  d: %f\n",
    //     potential[n], l, r, u, d);
    //  ROS_INFO("[Update] cost: %d\n", costs[n]);

    // 找到最小的potential，以及它相邻点的最小potential
    float ta, tc;
    if (l < r)
        tc = l;
    else
        tc = r;
    if (u < d)
        ta = u;
    else
        ta = d;

    float hf = cost; // 描述可通行性的因子
    float dc = tc - ta;        // tc和tc之间的代价之差
    if (dc < 0)         // 设置ta为最小的
    {
        dc = -dc;
        ta = tc;
    }

    // 计算新的potential, 方法二选一
    if (dc >= hf)        // ta 和 tc 的代价值差如果太大, 只用ta更新
        return ta + hf;
    else            // two-neighbor interpolation update
     //如果dc小于hf，则进行两个邻居的插值更新。通过二次近似来计算v值，然后返回ta加上hf乘以v作为新的潜在值
    {
        // use quadratic approximation
        // might speed this up through table lookup, but still have to
        //   do the divide
        float d = dc / hf;
        float v = -0.2301 * d * d + 0.5307 * d + 0.7040;
        return ta + hf * v;
    }
}
}

