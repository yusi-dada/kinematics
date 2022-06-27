#include <gtest/gtest.h>
#include "pose.h"
using namespace kinematics;
typedef vec3<double> vec3d;
typedef vec3<float> vec3f;
typedef vec4<double> vec4d;
typedef vec4<float> vec4f;
typedef pose<double> posed;
typedef pose<float> posef;

TEST(vec3, Test1)
{
    // クラス初期化
    vec3d a(1,2,3);
    vec3d b;
    EXPECT_TRUE( +a  == vec3d(1,2,3) );
    EXPECT_TRUE( -a  == vec3d(-1,-2,-3) );
    EXPECT_TRUE(  b  == vec3d(0,0,0) );

    // 要素アクセス
    EXPECT_EQ(1, a[0]);
    EXPECT_EQ(2, a[1]);
    EXPECT_EQ(3, a[2]);

    // 定数演算
    EXPECT_TRUE( 3+a == vec3d(3+1,3+2,3+3) );
    EXPECT_TRUE( a+2 == vec3d(2+1,2+2,2+3) );
    EXPECT_TRUE( 3-a == vec3d(3-1,3-2,3-3) );
    EXPECT_TRUE( a-2 == vec3d(1-2,2-2,3-2) );
    EXPECT_TRUE( 3*a == vec3d(3*1,3*2,3*3) );
    EXPECT_TRUE( a*2 == vec3d(2*1,2*2,2*3) );
    EXPECT_TRUE( 3/a == vec3d(3/1.0,3/2.0,3/3.0) );
    EXPECT_TRUE( a/2 == vec3d(1.0/2,2.0/2,3.0/2) );

    // 0割り
    EXPECT_FALSE( (a/0).isnan() );
    EXPECT_TRUE(  (a/0).isinf() );
    EXPECT_TRUE(  (-a/0).isinf() );
    EXPECT_TRUE(  (a/-0).isinf() );
    EXPECT_TRUE( (b/0).isnan() );
    EXPECT_TRUE( (-b/0).isnan() );
    EXPECT_FALSE( (b/0).isinf() );

    EXPECT_TRUE( vec3d(0,0,0).isnum() );
    EXPECT_FALSE( vec3d(INFINITY,0,0).isnum() );
    EXPECT_FALSE( vec3d(-INFINITY,0,0).isnum() );
    EXPECT_FALSE( vec3d(NAN,0,0).isnum() );
    EXPECT_FALSE( vec3d(-NAN,0,0).isnum() );
    EXPECT_FALSE( vec3d(NAN,INFINITY,0).isnum() );

    // 代入
    b = a;
    EXPECT_TRUE( a==b );    
    b = vec3f(1,2,3);
    EXPECT_TRUE( a==b );

    // クラス間演算
    EXPECT_TRUE( a+b == vec3d(2,4,6) );
    EXPECT_TRUE( a-b == vec3d(0,0,0) );
    EXPECT_TRUE( a*b == 14 );

    // 外積
    vec3d c(4,5,6);
    EXPECT_TRUE( a%b == vec3d( 0, 0, 0) );
    EXPECT_TRUE( a%c == vec3d(-3, 6,-3) );
    EXPECT_TRUE( c%a == vec3d( 3,-6, 3) );

    // ノルム
    EXPECT_TRUE( a.nrm() == sqrt(14) );

    vec3d d(0.001,0,-4);
    EXPECT_TRUE( d.iszero() == vec3d(0,1,0) );
    EXPECT_TRUE( d.sign() == vec3d(1,0,-1) );



}

TEST(vec4, Test1)
{
    // クラス初期化
    vec4d a;
    EXPECT_TRUE(a == vec4d(0,0,0,1));

    // クラス初期化＋要素アクセス
    vec4d b(1,2,3,4);
    EXPECT_EQ(1, b[0]);
    EXPECT_EQ(2, b[1]);
    EXPECT_EQ(3, b[2]);
    EXPECT_EQ(4, b[3]);

    // 単項演算子/共役
    EXPECT_TRUE(+b == vec4d( 1, 2, 3, 4));
    EXPECT_TRUE(-b == vec4d(-1,-2,-3,-4));
    EXPECT_TRUE(b.conj() == vec4d(-1,-2,-3, 4));
    EXPECT_TRUE(b == vec4d(1,2,3,4));   // conjで内部変数は変化しない

    // 正規化
    EXPECT_TRUE(b.nrm() == sqrt(30.0));
    EXPECT_TRUE(b.normalize() == vec4d(1/sqrt(30.0),2/sqrt(30.0),3/sqrt(30.0),4/sqrt(30.0)));

    // オイラー角による初期化/オイラー角演算
    double roll = 0.12345;
    double pitch = 0.67890;
    double yaw = 1.23456;
    vec4d c(roll, pitch, yaw);
    EXPECT_TRUE(  c.rpy() == vec3d(roll, pitch, yaw) );

    // 回転行列(q→C と rpy→Cの比較)
    auto C1 = c.C();
    auto C2 = rpy2C(roll, pitch, yaw);
    for(int i=0; i<3; i++)
    {
        EXPECT_TRUE( C1[i] == C2[i] );
        //std::cerr << "C1[" << i << "] = " << C1[i] << std::endl;
        //std::cerr << "C2[" << i << "] = " << C2[i] << std::endl;
    }

    // オイラーパラメータによる初期化
    vec4d d( vec3d(1,1,0), M_PI/6);
    vec4d e( vec3d(2,2,0), M_PI/6-2*M_PI);  //逆回転
    EXPECT_FALSE( d == e );
    EXPECT_TRUE(  d == -e );
    EXPECT_TRUE(  d.eq(d) );
    EXPECT_TRUE(  d.eq(e) );

    // 代入
    vec4d f = d;
    EXPECT_TRUE( f == d );

    // 変換
    vec4d g(vec3d(0,0,1),M_PI/6);
    EXPECT_TRUE( g.rpy() == vec3d(0,0,M_PI/6) );
    EXPECT_TRUE( g.Trans(vec3d(1,0,0)) == vec3d(cos(M_PI/6),sin(M_PI/6),0) );


}

TEST(pose, Test1)
{
    posed p;
    EXPECT_TRUE( p == posed(vec3d(0,0,0),vec4d(0,0,0)) );
    EXPECT_TRUE( p == posed(vec3d(0,0,0),vec4d(0,0,0,1)) );

    posed p1(vec3d(1.23,4.56,7.89),vec4d(0.987,0.654,0.321));
    posed p2(vec3d(0.987,0.654,0.321),vec4d(0.123,0.456,0.789));

    posed p3 = p1 * p2;
    posed p4 = p3 / p1;
    std::cerr << p2 << std::endl;
    std::cerr << p4 << std::endl;
    EXPECT_TRUE( p2 == p4 );
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}