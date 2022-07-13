#include <gtest/gtest.h>
#include <kinematics/kinematics.h>
using namespace kinematics;

TEST(pose, Test1)
{
    posed pos0;    // ベース座標系
    posed pos1;
    pos1.p = vec3d(0.12345, 0.23456, 0.34567);
    pos1.q = vec4d(0.12345, 0.23456, 0.34567);
    EXPECT_TRUE(pos1==pos0*pos1);   // ベース座標系からの変換は同じ

    posed pos12, pos23, pos14;
    posed pos2, pos3, pos4;
    pos12.p = vec3d(0.34567, 0.23456, 0.12345);
    pos12.q = vec4d(0.34567, 0.23456, 0.12345);
    pos23.p = vec3d(0.45678, 0.34567, 0.23456);
    pos23.q = vec4d(0.45678, 0.34567, 0.23456);
    pos14.p = vec3d(0.56789, 0.45678, 0.34567);
    pos14.q = vec4d(0.56789, 0.45678, 0.34567);
    pos2 = pos1 * pos12;
    pos3 = pos2 * pos23;
    pos4 = pos1 * pos14;    // pos1からpos3方向とは分岐

    posed pos31, pos32, pos43;
    pos31 = pos1 / pos3; // pos3 → pos1 の変換
    pos32 = pos2 / pos3; // pos3 → pos2 の変換
    pos43 = pos3 / pos4; // pos4 → pos3 の変換

    EXPECT_TRUE(pos1==pos3*pos31);
    EXPECT_TRUE(pos2==pos3*pos32);
    EXPECT_TRUE(pos3==pos4*pos43);

    
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}