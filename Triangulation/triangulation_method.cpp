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
    //std::cout << "mean x0 " << mean_x0 << std::endl;
    //std::cout << "mean y0: " << mean_y0 << std::endl;
    vec3 centroid = { mean_x, mean_y, 1.0 };
    //std::cout << "Centroid_0: " << centroid_0 << std::endl;

    //Translation of W by centroid
    // "" for points_0
    //std::cout << "points_0T: " << points_0T << std::endl;
    //std::cout << "points_0: " << points_0[0] << std::endl;
    float sum_mean_dist = 0;
    for (int i = 0; i < points.size(); i++)
    {
        sum_mean_dist = sum_mean_dist + (dist_c(points[i], centroid));
    }
    //std::cout << "sum dist0" << sum_mean_dist0 << std::endl;

    float mean_dist = sum_mean_dist / points.size();

    // Initialize scaling matrix
    trans = { ((float)sqrt(2.0) / mean_dist),0.0,-((float)sqrt(2.0) / mean_dist)*mean_x,
                   0.0,((float)sqrt(2.0) / mean_dist),-((float)sqrt(2.0) / mean_dist)* mean_y,
                   0.0,0.0,1.0 };
    //std::cout << "scaling matrix: " << scaling << std::endl;

    //Scaling of W 
    //For first point (test)
    //std::cout << "scaling one of the points_0T: " << scaling * points_0T[0]  << std::endl;
    //For all points

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
    /// NOTE: there might be multiple workflows for reconstructing 3D geometry from corresponding image points.
    ///       This assignment uses the commonly used one explained in our lecture.
    ///       It is advised to define a function for each sub-task. This way you have a clean and well-structured
    ///       implementation, which also makes testing and debugging easier. You can put your other functions above
    ///       triangulation(), or feel free to put them in one or multiple separate files.

    std::cout << "\nTODO: I am going to implement the triangulation() function in the following file:" << std::endl
        << "\t    - triangulation_method.cpp\n\n";

    std::cout << "[Liangliang]:\n"
        "\tFeel free to use any data structure and function offered by Easy3D, in particular the following two\n"
        "\tfiles for vectors and matrices:\n"
        "\t    - easy3d/core/mat.h  Fixed-size matrices and related functions.\n"
        "\t    - easy3d/core/vec.h  Fixed-size vectors and related functions.\n"
        "\tFor matrices with unknown sizes (e.g., when handling an unknown number of corresponding points\n"
        "\tstored in a file, where their sizes can only be known at run time), a dynamic-sized matrix data\n"
        "\tstructure is necessary. In this case, you can use the templated 'Matrix' class defined in\n"
        "\t    - Triangulation/matrix.h  Matrices of arbitrary dimensions and related functions.\n"
        "\tPlease refer to the corresponding header files for more details of these data structures.\n\n"
        "\tIf you choose to implement the non-linear method for triangulation (optional task). Please refer to\n"
        "\t'Tutorial_NonlinearLeastSquares/main.cpp' for an example and some explanations. \n\n"
        "\tIn your final submission, please\n"
        "\t    - delete ALL unrelated test or debug code and avoid unnecessary output.\n"
        "\t    - include all the source code (original code framework + your implementation).\n"
        "\t    - do NOT include the 'build' directory (which contains the intermediate files in a build step).\n"
        "\t    - make sure your code compiles and can reproduce your results without any modification.\n\n" << std::flush;

    /// Easy3D provides fixed-size matrix types, e.g., mat2 (2x2), mat3 (3x3), mat4 (4x4), mat34 (3x4).
    /// To use these matrices, their sizes should be known to you at the compile-time (i.e., when compiling your code).
    /// Once defined, their sizes can NOT be changed.
    /// In 'Triangulation/matrix.h', another templated 'Matrix' type is also provided. This type can have arbitrary
    /// dimensions and their sizes can be specified at run-time (i.e., when executing your program).
    /// Below are a few examples showing some of these data structures and related APIs.

    /// ----------- fixed-size matrices

    /// define a 3 by 4 matrix M (you can also define 3 by 4 matrix similarly)
    mat34 M(1.0f);  /// entries on the diagonal are initialized to be 1 and others to be 0.

    /// set the first row of M
    M.set_row(0, vec4(1, 1, 1, 1));    /// vec4 is a 4D vector.

    /// set the second column of M
    M.set_col(1, vec4(2, 2, 2, 2));

    /// get the 3 rows of M
    vec4 M1 = M.row(0);
    vec4 M2 = M.row(1);
    vec4 M3 = M.row(2);

    /// ----------- fixed-size vectors

    /// how to quickly initialize a std::vector
    std::vector<double> rows = { 0, 1, 2, 3,
                                4, 5, 6, 7,
                                8, 9, 10, 11 };
    /// get the '2'-th row of M
    const vec4 b = M.row(2);    // it assigns the requested row to a new vector b

    /// get the '1'-th column of M
    const vec3 c = M.col(1);    // it assigns the requested column to a new vector c

    /// modify the element value at row 2 and column 1 (Note the 0-based indices)
    M(2, 1) = b.x;

    /// apply transformation M on a 3D point p (p is a 3D vector)
    vec3 p(222, 444, 333);
    vec3 proj = M * vec4(p, 1.0f);  // use the homogenous coordinates. result is a 3D vector

    /// the length of a vector
    float len = p.length();
    /// the squared length of a vector
    float sqr_len = p.length2();

    /// the dot product of two vectors
    float dot_prod = dot(p, proj);

    /// the cross product of two vectors
    vec3 cross_prod = cross(p, proj);

    /// normalize this vector
    cross_prod.normalize();

    /// a 3 by 3 matrix (all entries are intentionally NOT initialized for efficiency reasons)
    //mat3 F;
    /// ... here you compute or initialize F.
    /// compute the inverse of K
    //mat3 invF = inverse(F);

    /// ----------- dynamic-size matrices

    /// define a non-fixed size matrix
    Matrix<double> W(2, 3, 0.0); // all entries initialized to 0.0.

    /// set its first row by a 3D vector (1.1, 2.2, 3.3)
    W.set_row({ 1.1, 2.2, 3.3 }, 0);   // here "{ 1.1, 2.2, 3.3 }" is of type 'std::vector<double>'

    /// get the last column of a matrix
    std::vector<double> last_column = W.get_column(W.cols() - 1);

    // TODO: delete all above demo code in the final submission

    //--------------------------------------------------------------------------------------------------------------
    // implementation starts ...

    // TODO: check if the input is valid (always good because you never known how others will call your function).
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

    // TODO: Estimate relative pose of two views. This can be subdivided into
    //      - estimate the fundamental matrix F;
    //      - compute the essential matrix E;
    //      - recover rotation R and t.

    //Estimation of fundamental matrix F;
    // Normalization of points
    
    mat3 trans_0;
    mat3 trans_1;

    std::vector<vec3> Npoints_0 = points_normalize(points_0, trans_0);
    std::vector<vec3> Npoints_1 = points_normalize(points_1, trans_1);

    /*
    for (int i = 0; i < points_0.size(); i++)
    {
        std::cout << points_0[i] << std::endl;
        std::cout << Npoints_0[i] << std::endl;
        std::cout << inverse(trans_0) * Npoints_0[i] << std::endl;
    }*/
    //std::cout <<  Npoints_0 * trans_0  << std::endl;

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
    //std::cout << "Normalized matrix: " << Wlol << std::endl;

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
    
    F(2, 2) = 1;
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

    std::cout << F << std::endl;
  
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

    //std::cout << EE << std::endl;

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
    std::cout << "detR1" << r1det << "detR2" << r2det << std::endl;
    std::cout <<"R1" << R1 << std:: endl;
    std::cout << "R2" <<R2 << std::endl;
    std::cout <<"t1" <<t1 << std::endl;
    std::cout << "t2" <<t2 << std::endl;
    
    //W*R*T
     
    mat3 R13 = to_mat3(R1);
    vec3 t13 { (float)t1[0], (float)t1[1], (float)t1[2] };
    mat3 R23 = to_mat3(R2);
    vec3 t23{ (float)t2[0], (float)t2[1], (float)t2[2] };

    //Finding the correct relative pose


    mat3 R13 = to_mat3(R1);
    vec3 t13{ (float)t1[0], (float)t1[1], (float)t1[2] };
    mat3 R23 = to_mat3(R2);
    vec3 t23{ (float)t2[0], (float)t2[1], (float)t2[2] };

    std::cout << "R13: " << R13 << std::endl;
    std::cout << "t13: " << t13 << std::endl;

    mat34 R1t1 = get_pose_M(R13, t13);
    mat34 R1t2 = get_pose_M(R13, t23);
    mat34 R2t1 = get_pose_M(R23, t13);
    mat34 R2t2 = get_pose_M(R23, t23);

    int count_p1 = 0;
    int count_p2 = 0;
    int count_p3 = 0;
    int count_p4 = 0;

    vec4 test_p{ points_1[0][0],points_1[0][1],points_1[0][2],1.0 };
    vec3 test_p_proj = R1t1 * test_p;
    std::cout << "R1t1: " << R1t1 << std::endl;
    std::cout << "test_p: " << test_p << std::endl;
    std::cout << "test: " << test_p_proj << std::endl;


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

    std::cout << "count_p1: " << count_p1 << std::endl;
    std::cout << "count_p2: " << count_p2 << std::endl;
    std::cout << "count_p3: " << count_p3 << std::endl;
    std::cout << "count_p4: " << count_p4 << std::endl;

    mat34 Mprime = K * R1t1;
    std::cout << "Mprime: " << Mprime << std::endl;
    Matrix<double> Mprimem = to_Matrix(Mprime);

    mat34 Mproj;
    Mproj.set_col(0, K.col(0));
    Mproj.set_col(1, K.col(1));
    Mproj.set_col(2, K.col(2));
    Mproj.set_col(3, vec3{ 0,0,0 });
    std::cout << Mproj << std::endl;
    Matrix<double> Mprojem = to_Matrix(Mproj);

    Matrix<double> PP(points_0.size(), 4);

    for (int i = 0; i < points_0.size(); i++) {
        Matrix<double> A(4, 3, 0.0);
        A.set_row((points_0[i][0] * Mprojem.get_row(2) - Mprojem.get_row(0)), 0);
        A.set_row((points_0[i][1] * Mprojem.get_row(2) - Mprojem.get_row(1)), 1);
        A.set_row((points_1[i][0] * Mprimem.get_row(2) - Mprimem.get_row(0)), 2);
        A.set_row((points_1[i][1] * Mprimem.get_row(2) - Mprimem.get_row(1)), 3);

        Matrix<double> AU(4,4);
        Matrix<double> AS(4,3);
        Matrix<double> AV(3,3);
        svd_decompose(A, AU, AS, AV);
        PP.set_column(AV.get_column(2), i);
        std::cout << "AV: " << AV << std::endl;

    }

    std::cout << "PP: " << PP << std::endl;

    /*
    // TODO: construct the P matrix (so P * m = 0).
    const int n = 9;
    const int m = 2 * (int)points_0.size();
    // Construction of  Matrix P
    Matrix<double> P(m, n, 0.0);
    for (int i = 0; i < points_0.size(); i++)
    {
        for (int j = 0; j < 2; j++)
        {
            for(int k = 0 ; k < 2; k++)
            {
            }
            P(4 * i+j , 3 * j) = -Mproj(i, 0);
            P(4 * i + j, 3 * j +1) = -Mproj(i, 1);
            P(4 * i + j, 3 * j + 2) = -Mproj(i, 2);
            P(4 * i + j+1, 3 * j) = -Mproj(i, 0);
            P(4 * i + j+1, 3 * j + 1) = -Mproj(i, 1);
            P(4 * i + j+1, 3 * j + 2) = -Mproj(i, 2);


        }
    }
    */

    // TODO: Reconstruct 3D points. The main task is
    //      - triangulate a pair of image points (i.e., compute the 3D coordinates for each corresponding point pair)

    //mat34 M_1(1.0f);                    // M = [I 0]




    // TODO: Don't forget to
    //          - write your recovered 3D points into 'points_3d' (the viewer can visualize the 3D points for you);
    //          - write the recovered relative pose into R and t (the view will be updated as seen from the 2nd camera,
    //            which can help you to check if R and t are correct).
    //       You must return either 'true' or 'false' to indicate whether the triangulation was successful (so the
    //       viewer will be notified to visualize the 3D points and update the view).
    //       However, there are a few cases you should return 'false' instead, for example:
    //          - function not implemented yet;
    //          - input not valid (e.g., not enough points, point numbers don't match);
    //          - encountered failure in any step.
    return points_3d.size() > 0;
}