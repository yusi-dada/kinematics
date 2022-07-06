#include <gtest/gtest.h>
#include <robot/bit.h>
//using namespace kinematics;


TEST(bit, Test1)
{
    unsigned int bit = 0b00101101;
    EXPECT_EQ(get_bit(bit, 0), true);
    EXPECT_EQ(get_bit(bit, 1), false);
    EXPECT_EQ(get_bit(bit, 2), true);
    EXPECT_EQ(get_bit(bit, 3), true);
    EXPECT_EQ(get_bit(bit, 4), false);
    EXPECT_EQ(get_bit(bit, 5), true);
    EXPECT_EQ(get_bit(bit, 6), false);
    EXPECT_EQ(get_bit(bit, 7), false);

    set_bit(bit,7);
    EXPECT_EQ(bit, 0b10101101);

    clr_bit(bit,1);
    EXPECT_EQ(bit, 0b10101101);

    clr_bit(bit,3);
    EXPECT_EQ(bit, 0b10100101);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}