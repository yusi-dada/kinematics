#include <gtest/gtest.h>
#include <robot/joint.h>
using namespace kinematics;


TEST(joint, Test1)
{
    // 初期化
    joint<double> a;
    joint<double> b = {1,2,3,4,5,6};
    EXPECT_TRUE( a == joint<double>({0,0,0,0,0,0}) );
    EXPECT_TRUE( +b == joint<double>({1,2,3,4,5,6}) );
    EXPECT_TRUE( -b == joint<double>({-1,-2,-3,-4,-5,-6}) );

    // 代入
    joint<double> c = b;
    EXPECT_TRUE( b == c);

    // 要素
    EXPECT_EQ( b[0], 1);    EXPECT_EQ( b[1], 2);    EXPECT_EQ( b[2], 3);
    EXPECT_EQ( b[3], 4);    EXPECT_EQ( b[4], 5);    EXPECT_EQ( b[5], 6);
    EXPECT_EQ( b[-1], 6);   EXPECT_EQ( b[-2], 5);   EXPECT_EQ( b[-3], 4);
    EXPECT_EQ( b[-4], 3);   EXPECT_EQ( b[-5], 2);   EXPECT_EQ( b[-6], 1);

    // 要素の変更
    b[1] = 5; 
    EXPECT_TRUE( b == joint<double>({1,5,3,4,5,6}) );

    // オブジェクト同士の和差
    EXPECT_TRUE( b-c == joint<double>({0,3,0,0,0,0}) );
    EXPECT_TRUE( b+c == joint<double>({2,7,6,8,10,12}) );
    b[1] = 2;   // もとに戻す b = {1,2,3,4,5,6}
    EXPECT_TRUE( b*c == joint<double>({1,4,9,16,25,36}) );
    EXPECT_TRUE( b/c == joint<double>({1,1,1,1,1,1}) );

    // 定数操作
    EXPECT_TRUE( 1+b == joint<double>({2,3,4,5,6,7}) );
    EXPECT_TRUE( b+2 == joint<double>({3,4,5,6,7,8}) );
    EXPECT_TRUE( 1-b == joint<double>({0,-1,-2,-3,-4,-5}) );
    EXPECT_TRUE( b-2 == joint<double>({-1,0,1,2,3,4}) );
    EXPECT_TRUE( 2*b == joint<double>({2,4,6,8,10,12}) );
    EXPECT_TRUE( b*2 == joint<double>({2,4,6,8,10,12}) );
    EXPECT_TRUE( 2/b == joint<double>({2.0/1.0,2.0/2.0,2.0/3.0,2.0/4.0,2.0/5.0,2.0/6.0}) );
    EXPECT_TRUE( b/2 == joint<double>({1.0/2.0, 2.0/2.0, 3.0/2.0, 4.0/2.0, 5.0/2.0, 6.0/2.0}) );

    // NAN, INF
    EXPECT_TRUE( (b/0).isinf() );
    EXPECT_FALSE( (b/0).isnan() );
    EXPECT_TRUE( (0*b/0).isnan() );
    EXPECT_FALSE( (0*b/0).isinf() );
    EXPECT_FALSE( (b/0).isnum() );
    EXPECT_FALSE( (0*b/0).isnum() );

    // SIGN, ABS, 0
    EXPECT_TRUE( (b-3).sign() == joint<double>({-1,-1,0,1,1,1})  );     // b-3 = {-2,-1,0,1,2,3}
    EXPECT_TRUE( (b-3).abs() == joint<double>({2,1,0,1,2,3})  );     // b-3 = {-2,-1,0,1,2,3}
    EXPECT_TRUE( (0*b).iszero() );
    EXPECT_FALSE( (b-3).iszero() );

    // SAT
    joint<double> inrange;
    joint<double> _min = {-0.9,-0.9,-0.9,-0.9,-0.9,-0.9};
    joint<double> _max = {1.2,1.2,1.2,1.2,1.2,1.2};
    EXPECT_TRUE( (b-3).sat(_min, _max, &inrange) == joint<double>({-0.9,-0.9,0,1,1.2,1.2})  );     // b-3 = {-2,-1,0,1,2,3}
    EXPECT_TRUE( inrange == joint<double>({0,0,1,1,0,0})  ); 
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}