#include <gtest/gtest.h>
#include <kinematics/kinematics.h>
using namespace kinematics;

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

    // 要素の変更
    a[0] = 4;
    a[2] = 2 * a[0];
    EXPECT_EQ(4, a[0]);
    EXPECT_EQ(8, a[2]);

    a[0] = 1;
    a[2] = 3;

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

    vec3d d(0.001, 0, -4);
    EXPECT_TRUE( d.iszero() == vec3d(0,1,0) );
    EXPECT_TRUE( d.sign() == vec3d(1,0,-1) );
    
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}