/**
 * Copyright (C) 2015 by Liangliang Nan (liangliang.nan@gmail.com)
 * https://3d.bk.tudelft.nl/liangliang/
 *
 * This file is part of Easy3D. If it is useful in your research/work,
 * I would be grateful if you show your appreciation by citing it:
 * ------------------------------------------------------------------
 *      Liangliang Nan.
 *      Easy3D: a lightweight, easy-to-use, and efficient C++
 *      library for processing and rendering 3D data. 2018.
 * ------------------------------------------------------------------
 * Easy3D is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 3
 * as published by the Free Software Foundation.
 *
 * Easy3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "triangulation.h"
#include "matrix_algo.h"
#include <easy3d/optimizer/optimizer_lm.h>


using namespace easy3d;

/// Function to calculate distance between centroid and input point
float dist_c(vec3 point, vec3 centroid)
{
    vec3 diff = point - centroid;
    float dist = diff.length();
    return dist;
}

/// Normalize data points
std::vector<vec3> points_normalize(std::vector<vec3> points, mat3 &trans)
{
    double sum_x = 0;
    double sum_y = 0;
    for (int i = 0; i < points.size(); i++)
    {
        sum_x = sum_x + points[i][0];
        sum_y = sum_y + points[i][1];
    }
    float mean_x = sum_x / points.size();
    float mean_y = sum_y / points.size();

    // Centroid
    vec3 centroid = { mean_x, mean_y, 1.0 };

    //Translation of W by centroid
    float sum_mean_dist = 0;
    for (int i = 0; i < points.size(); i++)
    {
        sum_mean_dist = sum_mean_dist + (dist_c(points[i], centroid));
    }

    float mean_dist = sum_mean_dist / points.size();

    // Initialize scaling matrix
    trans = { ((float)sqrt(2.0) / mean_dist),0.0,-((float)sqrt(2.0) / mean_dist)*mean_x,
                   0.0,((float)sqrt(2.0) / mean_dist),-((float)sqrt(2.0) / mean_dist)* mean_y,
                   0.0,0.0,1.0 };

    //Scaling of W 

    std::vector<vec3> points_scaled;

    for (int i = 0; i < points.size(); i++)
    {
        vec3 scaled_point = trans * points[i];
        points_scaled.push_back(scaled_point);
    }

    return points_scaled;
}
// Figure out the poses
mat34 get_pose_M(mat3 R, vec3 t) {
    mat34 Rt;
    Rt.set_col(0, R.col(0));
    Rt.set_col(1, R.col(1));
    Rt.set_col(2, R.col(2));
    Rt.set_col(3, t);
    return Rt;
}


/// convert a 3 by 3 matrix of type 'Matrix<double>' to mat3
mat3 to_mat3(Matrix<double>& M) {
    mat3 result;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j)
            result(i, j) = M(i, j);
    }
    return result;
}


/// convert M of type 'matN' (N can be any positive integer) to type 'Matrix<double>'
template<typename mat>
Matrix<double> to_Matrix(const mat& M) {
    const int num_rows = M.num_rows();
    const int num_cols = M.num_columns();
    Matrix<double> result(num_rows, num_cols);
    for (int i = 0; i < num_rows; ++i) {
        for (int j = 0; j < num_cols; ++j)
            result(i, j) = M(i, j);
    }
    return result;
}


/**
 * TODO: Finish this function for reconstructing 3D geometry from corresponding image points.
 * @return True on success, otherwise false. On success, the reconstructed 3D points must be written to 'points_3d'.
 */
bool Triangulation::triangulation(
    float fx, float fy,     /// input: the focal lengths (same for both cameras)
    float cx, float cy,     /// input: the principal point (same for both cameras)
    const std::vector<vec3>& points_0,    /// input: image points (in homogenous coordinates) in the 1st image.
    const std::vector<vec3>& points_1,    /// input: image points (in homogenous coordinates) in the 2nd image.
    std::vector<vec3>& points_3d,         /// output: reconstructed 3D points
    mat3& R,   /// output: recovered rotation of 2nd camera (used for updating the viewer and visual inspection)
    vec3& t    /// output: recovered translation of 2nd camera (used for updating the viewer and visual inspection)
) const
{
    // CHECK FOR VALIDITY INPUTS
    /*float fx, float fy,     /// input: the focal lengths (same for both cameras)
        float cx, float cy,     /// input: the principal point (same for both cameras)
        const std::vector<vec3>& points_0,    /// input: image points (in homogenous coordinates) in the 1st image.
        const std::vector<vec3>& points_1,    /// input: image points (in homogenous coordinates) in the 2nd image.
    */
    assert(points_0.size() >= 8);
    assert(points_1.size() >= 8);
    assert(points_0.size() == points_1.size());
    for (int i = 0; i < points_0.size(); i++)
    {
        assert(points_0[i].size() == 3);
        assert(points_0[i][2] == 1);
    }
    for (int i = 0; i < points_1.size(); i++)
    {
        assert(points_1[i].size() == 3);
        assert(points_1[i][2] == 1);
    }
    
    //Estimation of fundamental matrix F;
    // Normalization of points
    
    mat3 trans_0;
    mat3 trans_1;

    std::vector<vec3> Npoints_0 = points_normalize(points_0, trans_0);
    std::vector<vec3> Npoints_1 = points_normalize(points_1, trans_1);


    //Construction of W 
    //Initialize empty matrix W
    Matrix<double> Wlol(Npoints_0.size(), 9, 0.0);

    //std::vector<vec3> points_1scaled;


    for (int i = 0; i < Npoints_0.size(); i++)
    {
        double x1 = Npoints_0[i][0];
        double x2 = Npoints_1[i][0];
        double y1 = Npoints_0[i][1];
        double y2 = Npoints_1[i][1];
        std::vector<double> longvec{ x1 * x2, y1 * x2, x2, x1 * y2, y1 * y2, y2, x1, y1, 1.0 };
        Wlol.set_row(longvec, i);
    }

    //Linear solution (based on SVD)
    
    int m = points_0.size();
    int n = 9;
    Matrix<double> U(m, m, 0.0);   // initialized with 0s
    Matrix<double> S(m, n, 0.0);   // initialized with 0s
    Matrix<double> V(n, n, 0.0);   // initialized with 0s
    svd_decompose(Wlol, U, S, V);
    
    std::vector<double> fv = V.get_column(V.cols()-1);
    mat3 F;                         //Fundamental Matrix 
    int e = 0;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            F(i, j) =  fv[ e ];     //0-8
            e++;
        }
    }
    
    int nf = 3;
    Matrix<double> UF(nf, nf, 0.0);   // initialized with 0s
    Matrix<double> SF(nf, nf, 0.0);   // initialized with 0s
    Matrix<double> VF(nf, nf, 0.0);   // initialized with 0s
    
    Matrix<double> FF = to_Matrix(F);

    svd_decompose(FF, U, S, V);
    //Constraint enforcement(based on SVD)
    S(2, 2) = 0;
    FF = U * S * transpose(V);
    //Denormalization
    F = to_mat3(FF);
    F = transpose(trans_1) * F * trans_0;
  
    //STEP 2

   //Find the 4 relative poses 

    //Compute matrix E = KtFK

    // Matrix K
    mat3 K { fx,1.0,cx,
              0.0,fy,cy,
              0.0,0.0,1.0 };

    mat3 E;

    E = transpose(K) * F * K; //(2.1) 

    mat3 WW{ 0.0,-1.0, 0.0, //(2.2)
            1.0,0.0,0.0,
            0.0,0.0,1.0 };

    mat3 ZZ{ 0.0,1.0, 0.0, //(2.2)
            -1.0,0.0,0.0,
            0.0,0.0,0.0 };

    Matrix<double> Wd = to_Matrix(WW);
    Matrix<double> Zd = to_Matrix(ZZ);

    Matrix<double> UE(nf, nf, 0.0);   // initialized with 0s
    Matrix<double> SE(nf, nf, 0.0);   // initialized with 0s
    Matrix<double> VE(nf, nf, 0.0);   // initialized with 0s

    Matrix<double> EE = to_Matrix(E);

    svd_decompose(EE, UE, SE, VE);

    SE(0, 0) = 1.0; SE(1, 1) = 1.0;

    EE = UE * SE * transpose(VE);

    
    //Options of R an t
   
    Matrix<double> R1;
    Matrix<double> R2;
    std::vector<double> t1;
    std::vector<double> t2;
    
    double r1det = determinant(UE * Wd * transpose(VE));        //(2.7)
    double r2det = determinant(UE * transpose(Wd) * transpose(VE));
    R1 = r1det * (UE * Wd * transpose(VE));
    R2 = r2det * (UE * transpose(Wd) * transpose(VE));

    t1 = UE.get_column(2);                                      //(2.9)
    t2 = -1.0* UE.get_column(2);
    
    //Finding the correct relative pose

    mat3 R13 = to_mat3(R1);
    vec3 t13{ (float)t1[0], (float)t1[1], (float)t1[2] };
    mat3 R23 = to_mat3(R2);
    vec3 t23{ (float)t2[0], (float)t2[1], (float)t2[2] };

    // Get the relative poses
    mat34 R1t1 = get_pose_M(R13, t13);
    mat34 R1t2 = get_pose_M(R13, t23);
    mat34 R2t1 = get_pose_M(R23, t13);
    mat34 R2t2 = get_pose_M(R23, t23);

    //count the points in front of the cameras by its pose.

    int count_p1 = 0;
    int count_p2 = 0;
    int count_p3 = 0;
    int count_p4 = 0;

    vec4 test_p{ points_1[0][0],points_1[0][1],points_1[0][2],1.0 };
    vec3 test_p_proj = R1t1 * test_p;


    for (int i = 0; i < points_1.size(); i++)
    {
        vec4 p{ Npoints_1[i][0],Npoints_1[i][1],Npoints_1[i][2],1.0f };
        auto p1 = R1t1 * p;
        if (p1[2] >= 0) count_p1++;
        auto p2 = R1t2 * p;
        if (p2[2] >= 0) count_p2++;
        auto p3 = R2t1 * p;
        if (p3[2] >= 0) count_p3++;
        auto p4 = R2t2 * p;
        if (p4[2] >= 0) count_p4++;
    }
    
    //Get the pose

    mat34 RT;
    mat3 RR;
    vec3 tt;

    int i = std::max(std::max(std::max(count_p1, count_p2), count_p3), count_p4);
    if (i == count_p1)
    {
        RT = R1t1;
        RR = R13;
        tt = t13;
    }
    else if (i == count_p2)
    {
        RT = R1t2;
        RR = R13;
        tt = t23;
    }
    else if (i == count_p3)
    {
        RT = R2t1;
        RR = R23;
        tt = t13;
    }
    else
    {
        RT = R2t2;
        RR = R23;
        tt = t23;
    }
    // Make the matrices M

    mat34 Mprime = K * RT;
    Matrix<double> Mprimem = to_Matrix(Mprime);

    mat34 Mproj;
    Mproj.set_col(0, K.col(0));
    Mproj.set_col(1, K.col(1));
    Mproj.set_col(2, K.col(2));
    Mproj.set_col(3, vec3{ 0,0,0 });
    Matrix<double> Mprojem = to_Matrix(Mproj);

    //Calculate 3d points

    for (int i = 0; i < points_0.size(); i++) {
        Matrix<double> A(4, 4, 0.0);
        
        std::vector<double> s1 = (points_0[i][0] * Mprojem.get_row(2) - Mprojem.get_row(0));
        std::vector<double> s2 = (points_0[i][1] * Mprojem.get_row(2) - Mprojem.get_row(1));
        std::vector<double> s3 = (points_1[i][0] * Mprimem.get_row(2) - Mprimem.get_row(0));
        std::vector<double> s4 = (points_1[i][1] * Mprimem.get_row(2) - Mprimem.get_row(1));
        A.set_row({s1[0],s1[1],s1[2],s1[3]}, 0);
        A.set_row({ s2[0],s2[1],s2[2],s2[3] }, 1);
        A.set_row({ s3[0],s3[1],s3[2],s3[3] }, 2);
        A.set_row({ s4[0],s4[1],s4[2],s4[3] }, 3);
        Matrix<double> AU(4,4,0.0);
        Matrix<double> AS(4,4,0.0);
        Matrix<double> AV(4,4,0.0);
        svd_decompose(A, AU, AS, AV);
        vec4 p{ (float) AV(0,3), (float) AV(1,3), (float)AV(2,3), (float) AV(3,3) };
        vec3 pt{ p[0] / p[3],p[1] / p[3], p[2] / p[3] };
        points_3d.push_back(pt);
    }
    R = RR;
    t = tt;
    return points_3d.size() > 0;
}