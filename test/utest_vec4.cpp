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
    EXPECT_TRUE(b == vec4d(1,2,3,4));       // conjで内部変数は変化しない

    // 正規化
    EXPECT_TRUE(b.nrm() == sqrt(30.0));
    EXPECT_TRUE(b.normalize() == vec4d(1/sqrt(30.0),2/sqrt(30.0),3/sqrt(30.0),4/sqrt(30.0)));
    EXPECT_FALSE(b == vec4d( 1, 2, 3, 4));  // normalizeで内部変数変化

    // 代入
    vec4d c(1,2,3,4);
    vec4d d = c;
    EXPECT_TRUE( d == c );

    // 型違い代入
    vec4f e(1,2,3,4);
    d = e;
    EXPECT_TRUE( d == c );

    // 一致判定
    vec4d f(1,2,3,4);
    vec4d g = -f;
    EXPECT_FALSE( f==g );       // 成分の一致
    EXPECT_TRUE( f.eq(g) );     // 同一回転
    
    // nan,inf
    vec4d h(NAN,0,INFINITY,0);
    EXPECT_TRUE(h.isnan());
    EXPECT_TRUE(h.isinf());
    EXPECT_FALSE(h.isnum());

    vec4d i(-INFINITY,10,INFINITY,0);
    EXPECT_TRUE(i.isinf());
    EXPECT_FALSE(i.isnan());
    EXPECT_FALSE(i.isnum());
}

TEST(vec4, Test2)
{
    // 行列
    std::vector<double> angle_list = {0, M_PI/6, M_PI/4, M_PI/3, M_PI/2, 3*M_PI/4, 2*M_PI/3, 5*M_PI/6, M_PI};
//    std::vector<double> angle_list = {-M_PI, -M_PI/2, 0, M_PI/2, M_PI};
    int list_size = angle_list.size();

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
                mat3d C = rpy2C(roll, pitch, yaw);
                vec4d q(C);
                EXPECT_TRUE(C==q.C()) << "rpy= " << vec3d(roll, pitch, yaw) << "\n"
                                      << C << "\n" << q.C()<< "\n";  // rpy2C, vec4.C()
            }
        }
    }


}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}