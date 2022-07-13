#include <gtest/gtest.h>
#include <robot/sflg.h>
using namespace kinematics;

TEST(sflg1, Test1)
{
    SFLG1 sflg1;
    sflg1.val = 7;

    sflg1.SET(0, false);
    EXPECT_EQ(sflg1.val, 0b110);
    sflg1.SET(0, true);
    EXPECT_EQ(sflg1.val, 0b111);

    sflg1.SET(1, false);
    EXPECT_EQ(sflg1.val, 0b101);
    sflg1.SET(1, true);
    EXPECT_EQ(sflg1.val, 0b111);

    sflg1.SET(2, false);
    EXPECT_EQ(sflg1.val, 0b011);
    sflg1.SET(2, true);
    EXPECT_EQ(sflg1.val, 0b111);

    SFLG2 sflg2;
    for(int i=0; i<8; i++)
    {
        sflg2.SET(i, -4+i);
        std::cerr << sflg2 << std::endl;
    }
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}