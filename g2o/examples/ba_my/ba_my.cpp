// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <iostream>
#include <unordered_set>

#include "g2o/core/solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
//#include "g2o/math_groups/se3quat.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include "g2o/stuff/sampler.h"
#include "optimizer.h"

using namespace Eigen;
using namespace std;

class Sample {
public:
    static int uniform(int from, int to) { return static_cast<int>(g2o::Sampler::uniformRand(from, to)); }
};


vector<Vector3d> generate_points(size_t num_points);

bool is_projected_inside_frame(const Vector2d &z);

g2o::EdgeProjectXYZ2UV *createProjectionEdge(bool ROBUST_KERNEL,
                                             g2o::VertexSBAPointXYZ *v_p, const Vector2d &z,
                                             g2o::HyperGraph::Vertex *pVertex);

Vector3d
getDiff(const shared_ptr<g2o::SparseOptimizer> &optimizer, const vector<Vector3d> &true_points, const int pointid,
        const int trueid);

double calcSumDiff(const shared_ptr<g2o::SparseOptimizer> &optimizer, const vector<Vector3d> &true_points,
                   const unordered_map<int, int> &pointid_2_trueid, const unordered_set<int> &inliers);

int num_observations(const Matrix<double, 3, 1> &cur_true_point,
                     const vector<g2o::SE3Quat, aligned_allocator<g2o::SE3Quat>> &true_poses,
                     const g2o::CameraParameters *cam_params) {
    int num_obs = 0;
    for (size_t j = 0; j < true_poses.size(); ++j) {
        Vector2d z = cam_params->cam_map(true_poses.at(j).map(cur_true_point));
        if (z[0] >= 0 && z[1] >= 0 && z[0] < 640 && z[1] < 480) {
            ++num_obs;
        }
    }
    return num_obs;
}

g2o::HyperGraph::Vertex *
getVertexById(const g2o::SparseOptimizer *optimizer, size_t j) { return optimizer->vertices().find(j)->second; }


int main(int argc, const char *argv[]) {
    if (argc < 2) {
        cout << endl;
        cout << "Please type: " << endl;
        cout << "ba_demo [PIXEL_NOISE] [OUTLIER RATIO] [ROBUST_KERNEL] [STRUCTURE_ONLY] [DENSE]" << endl;
        cout << endl;
        cout << "PIXEL_NOISE: noise in image space (E.g.: 1)" << endl;
        cout << "OUTLIER_RATIO: probability of spuroius observation  (default: 0.0)" << endl;
        cout << "ROBUST_KERNEL: use robust kernel (0 or 1; default: 0==false)" << endl;
        cout
                << "STRUCTURE_ONLY: performe structure-only BA to get better point initializations (0 or 1; default: 0==false)"
                << endl;
        cout << "DENSE: Use dense solver (0 or 1; default: 0==false)" << endl;
        cout << endl;
        cout << "Note, if OUTLIER_RATIO is above 0, ROBUST_KERNEL should be set to 1==true." << endl;
        cout << endl;
        exit(0);
    }

    double PIXEL_NOISE = atof(argv[1]);
    double OUTLIER_RATIO = 0.0;

    if (argc > 2) {
        OUTLIER_RATIO = atof(argv[2]);
    }

    bool ROBUST_KERNEL = false;
    if (argc > 3) {
        ROBUST_KERNEL = atoi(argv[3]) != 0;
    }
    bool STRUCTURE_ONLY = false;
    if (argc > 4) {
        STRUCTURE_ONLY = atoi(argv[4]) != 0;
    }

    bool DENSE = false;
    if (argc > 5) {
        DENSE = atoi(argv[5]) != 0;
    }

    cout << "PIXEL_NOISE: " << PIXEL_NOISE << endl;
    cout << "OUTLIER_RATIO: " << OUTLIER_RATIO << endl;
    cout << "ROBUST_KERNEL: " << ROBUST_KERNEL << endl;
    cout << "STRUCTURE_ONLY: " << STRUCTURE_ONLY << endl;
    cout << "DENSE: " << DENSE << endl;


    bool verbose = false;
    shared_ptr<g2o::SparseOptimizer> optimizer = getOptimizer(DENSE, verbose);


    vector<Vector3d> true_points = generate_points(500);

    double focal_length = 1000.;
    Vector2d principal_point(320., 240.);

    vector<g2o::SE3Quat, aligned_allocator<g2o::SE3Quat> > true_poses;
    g2o::CameraParameters *cam_params = new g2o::CameraParameters(focal_length, principal_point, 0.);
    cam_params->setId(0);

    if (!optimizer->addParameter(cam_params)) {
        assert(false);
    }

    int vertex_id = 0;
    for (size_t i = 0; i < 15; ++i) {
        Vector3d trans(i * 0.04 - 1., 0, 0);

        Eigen::Quaterniond q;
        q.setIdentity();
        g2o::SE3Quat pose(q, trans);
        g2o::VertexSE3Expmap *v_se3 = new g2o::VertexSE3Expmap();
        v_se3->setId(vertex_id);
        if (i < 2) {
            v_se3->setFixed(true);
        }
        v_se3->setEstimate(pose);
        optimizer->addVertex(v_se3);
        true_poses.push_back(pose);
        vertex_id++;
    }
    int point_id = vertex_id;
    int point_num = 0;

    cout << endl;
    unordered_map<int, int> pointid_2_trueid;
    unordered_set<int> inliers;

    for (size_t i = 0; i < true_points.size(); ++i) {
        g2o::VertexSBAPointXYZ *v_p
                = new g2o::VertexSBAPointXYZ();
        v_p->setId(point_id);
        v_p->setMarginalized(true);
        v_p->setEstimate(true_points.at(i)
                         + Vector3d(g2o::Sampler::gaussRand(0., 1),
                                    g2o::Sampler::gaussRand(0., 1),
                                    g2o::Sampler::gaussRand(0., 1)));
        Matrix<double, 3, 1> &cur_true_point = true_points.at(i);
        int num_obs = num_observations(cur_true_point, true_poses, cam_params);

        if (num_obs >= 2) {
            optimizer->addVertex(v_p);
            bool inlier = true;
            for (size_t j = 0; j < true_poses.size(); ++j) {
                Vector2d z = cam_params->cam_map(true_poses.at(j).map(true_points.at(i)));

                if (is_projected_inside_frame(z)) {
                    double sam = g2o::Sampler::uniformRand(0., 1.);
                    if (sam < OUTLIER_RATIO) {
                        z = Vector2d(Sample::uniform(0, 640),
                                     Sample::uniform(0, 480));
                        inlier = false;
                    }
                    z += Vector2d(g2o::Sampler::gaussRand(0., PIXEL_NOISE),
                                  g2o::Sampler::gaussRand(0., PIXEL_NOISE));
                    g2o::HyperGraph::Vertex *pVertex = getVertexById(optimizer.get(), j);
                    g2o::EdgeProjectXYZ2UV *e = createProjectionEdge(ROBUST_KERNEL, v_p, z, pVertex);
                    optimizer->addEdge(e);
                }
            }

            if (inlier) {
                inliers.insert(point_id);
            }
            pointid_2_trueid.insert(make_pair(point_id, i));
            ++point_id;
            ++point_num;
        }
    }

    double sum_diff_ = calcSumDiff(optimizer, true_points, pointid_2_trueid, inliers);

    cout << endl;
    optimizer->initializeOptimization();
    optimizer->setVerbose(true);

    optimizer->save("my_ba_before_optimization.g2o");
    cout << endl;
    cout << "Performing full BA:" << endl;
    optimizer->optimize(10);

    optimizer->save("my_ba_after_optimization.g2o");
    cout << endl;
    cout << "Point error before optimisation (inliers only): " << sqrt(sum_diff_ / inliers.size()) << endl;

    auto sum_diff2_after = calcSumDiff(optimizer, true_points, pointid_2_trueid, inliers);
    cout << "Point error after optimisation (inliers only): " << sqrt(sum_diff2_after / inliers.size()) << endl;
    cout << endl;
}

double calcSumDiff(const shared_ptr<g2o::SparseOptimizer> &optimizer, const vector<Vector3d> &true_points,
                   const unordered_map<int, int> &pointid_2_trueid, const unordered_set<int> &inliers) {
    const double sum_diff_ = accumulate(
            pointid_2_trueid.begin(), pointid_2_trueid.end(), 0.0,
            [&optimizer, &true_points, &inliers](double sum, pair<int, int> pointid_trueid) {
                if (inliers.find(pointid_trueid.first) == inliers.end()) {
                    return sum;
                }
                Vector3d diff = getDiff(optimizer, true_points, pointid_trueid.first, pointid_trueid.second);
                return sum + diff.dot(diff);
            }
    );
    return sum_diff_;
}

Vector3d
getDiff(const shared_ptr<g2o::SparseOptimizer> &optimizer, const vector<Vector3d> &true_points, const int pointid,
        const int trueid) {
    g2o::HyperGraph::Vertex *vertex = getVertexById(optimizer.get(), pointid);
    auto true_point = true_points[trueid];
    g2o::VertexSBAPointXYZ *vertex_cast = dynamic_cast<g2o::VertexSBAPointXYZ *>(vertex);
    assert(vertex_cast != nullptr);
//    cout << vertex_cast->estimate() << "\n";
//    cout << true_point << "\n";
    Vector3d diff = vertex_cast->estimate() - true_point;
    return diff;
}


g2o::EdgeProjectXYZ2UV *createProjectionEdge(bool ROBUST_KERNEL, g2o::VertexSBAPointXYZ *v_p, const Vector2d &z,
                                             g2o::HyperGraph::Vertex *pVertex) {
    g2o::EdgeProjectXYZ2UV *e = new g2o::EdgeProjectXYZ2UV();
    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(v_p));
    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(pVertex));
    e->setMeasurement(z);
    e->information() = Matrix2d::Identity();
    if (ROBUST_KERNEL) {
        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
        e->setRobustKernel(rk);
    }
    e->setParameterId(0, 0);
    return e;
}

bool is_projected_inside_frame(const Vector2d &z) { return z[0] >= 0 && z[1] >= 0 && z[0] < 640 && z[1] < 480; }


vector<Vector3d> generate_points(size_t num_points) {
    vector<Vector3d> true_points;
    for (size_t i = 0; i < num_points; ++i) {
        true_points.push_back(Vector3d((g2o::Sampler::uniformRand(0., 1.) - 0.5) * 3,
                                       g2o::Sampler::uniformRand(0., 1.) - 0.5,
                                       g2o::Sampler::uniformRand(0., 1.) + 3));
    }
    return true_points;
}

