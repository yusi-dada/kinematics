#include <gtest/gtest.h>
#include <kinematics/kinematics.h>
using namespace kinematics;


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

    // 回転行列
    auto C1 = c.C();
    for(int i=0; i<3; i++)
        std::cerr << "C1[" << i << "] = " << C1[i] << std::endl;

    vec4d c2(C1);
    std::cerr << "c  = " << c << std::endl;
    std::cerr << "c2 = " << c2 << std::endl;
    EXPECT_TRUE(  c == c2 );

    // オイラーパラメータによる初期化
    vec4d d( vec3d(1,1,0), M_PI/6);         // [-pi pi]
    vec4d e( vec3d(2,2,0), M_PI/6+4*M_PI);  // [-pi pi]の範囲外
//    EXPECT_FALSE(  d == e );  // 回転方向による違いはなし
//    EXPECT_TRUE( d == -e );
    EXPECT_TRUE(  d.eq(e) );
    EXPECT_TRUE(  d.eq(-e) );   // 回転方向の区別なしで比較

    // 代入
    vec4d f = d;
    EXPECT_TRUE( f == d );

    // 変換
    vec4d g(vec3d(0,0,1),M_PI/6);
    EXPECT_TRUE( g.rpy() == vec3d(0,0,M_PI/6) );
    EXPECT_TRUE( g.Trans(vec3d(1,0,0)) == vec3d(cos(M_PI/6),sin(M_PI/6),0) );
}

TEST(vec4, Test2)
{
    // オイラーパラメータ <===> 方向余弦行列

    // 角度リスト作成
    int list_size = 3;
    std::vector<double> angle_list(list_size);
    for(int i=0; i<list_size; i++)
        angle_list[i] = -M_PI/3 + (M_PI/2.0)*i;

    double roll, pitch, yaw;
    for(int i=0; i<list_size; i++)
    {
        roll = angle_list[i];
        for(int j=0; j<list_size; j++)
        {
            pitch = angle_list[j];
            for(int k=0; k<list_size; k++)
            {
                yaw = angle_list[k];
                vec4d q(roll, pitch, yaw);          // EulerParameter (from rpy)
                auto C1 = q.C();
                auto C2 = rpy2C(roll, pitch, yaw);
//                std::cerr << C1 << std::endl;
//                std::cerr << C2 << std::endl;
//                std::cerr << "------------" << std::endl;
                EXPECT_TRUE(C1==C2);
            }
        }
    }



}

TEST(vec4, Test3)
{
    // オイラーパラメータ <===> オイラー角

    // 角度リスト作成
    int list_size = 3;
    std::vector<double> angle_list(list_size);
    for(int i=0; i<list_size; i++)
        angle_list[i] = -M_PI/3 + (M_PI/2.0)*i;
//        angle_list[i] = -2.0*M_PI + (M_PI/2.0)*i;

    double roll, pitch, yaw;
    for(int i=0; i<list_size; i++)
    {
        roll = angle_list[i];
        for(int j=0; j<list_size; j++)
        {
            pitch = angle_list[j];
            for(int k=0; k<list_size; k++)
            {
                yaw = angle_list[k];
                vec3d rpy(roll, pitch, yaw);        // rpy (input)
                vec4d q(roll, pitch, yaw);          // EulerParameter (from rpy)
                vec3d rpy2 = q.rpy();               // rpy2 (from EulerParameter)
                vec4d q2(rpy2.x, rpy2.y, rpy2.z);   // EulerParameter2 (from rpy2)
                bool test1 = (rpy == rpy2);
                bool test2 = (q.eq(q2));        // 同一回転  
                //bool test2 = (q == q2);       // 成分完全一致
                char cData[512];
                sprintf(cData, "rpy1[deg]=(%3.0f, %3.0f, %3.0f) --> %d\nrpy2[deg]=(%3.0f, %3.0f, %3.0f)", 
                        roll*180/M_PI, pitch*180/M_PI, yaw*180/M_PI, test1 & test2,
                        rpy2.x*180/M_PI, rpy2.y*180/M_PI, rpy2.z*180/M_PI);
                std::cerr << cData << std::endl;
                //EXPECT_TRUE(test2) << q << q2;
                
            }
        }
    }

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