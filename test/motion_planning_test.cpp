#include "Common.hpp"
#include <gtest/gtest.h>

class MotionPlanningUnitTests: public ::testing::Test {
    public:
        MotionPlanningUnitTests() {}
        ~MotionPlanningUnitTests() {}
};

// 浮点数比较测试
TEST_F(MotionPlanningUnitTests, valueTest) {
    EXPECT_TRUE(Tools::isZero(1e-8)) << "zero test is run";
    EXPECT_TRUE(Tools::isLarge(1.001, 1.000)) << "large test is run";
    EXPECT_TRUE(Tools::isSmall(1.000, 1.001)) << "small test is run";
    EXPECT_TRUE(Tools::isEqual(1.0 + 1e-9, 1.0 - 1e-9)) << "equal test is run";
}

// 区间相交测试
TEST_F(MotionPlanningUnitTests, interactTest) {
     // 定义第一个区间
    Section axis_section;
    SectionSet axis_section_set_1;
    axis_section = {2, 8};
    axis_section_set_1.push_back(axis_section);
    axis_section = {9, 14};
    axis_section_set_1.push_back(axis_section);
    axis_section = {15, 20};
    axis_section_set_1.push_back(axis_section);
    axis_section = {22, 25.0};
    axis_section_set_1.push_back(axis_section);

    // 定义第二个区间
    SectionSet axis_section_set_2;
    axis_section = {1, 10};
    axis_section_set_2.push_back(axis_section);
    axis_section = {12, 16};
    axis_section_set_2.push_back(axis_section);
    axis_section = {19, 25};
    axis_section_set_2.push_back(axis_section);
    axis_section = {24.9, 40};
    axis_section_set_2.push_back(axis_section);

    // 求交集,结果应该是{2,8;9,10;12,14;15,16;19,20;22,25}
    SectionSet intersection = Tools::getIntersection(axis_section_set_1, axis_section_set_2);
    EXPECT_EQ(intersection.size(), 6) << "interact size test is run";
    EXPECT_TRUE(Tools::isEqual(intersection[intersection.size() - 1].max_, 25.0)) << "large boundary test";
    EXPECT_TRUE(Tools::isEqual(intersection[0].min_, 2.0)) << "small boundary test";
}

// 线段相交测试
TEST_F(MotionPlanningUnitTests, lineSegmentInteractTest) {
    path_planning_msgs::PathPoint p1, p2, p3, p4;
    p1.x = 0;
    p1.y = 0;
    p2.x = 3;
    p2.y = 3;
    p3.x = 2;
    p3.y = 2;
    p4.x = 4;
    p4.y = 4;
    bool flag = false;
    flag = Tools::isLineSegmentInteracted(p1, p2, p3, p4);
    EXPECT_FALSE(flag) << "line segment1 interact test";
    p1.x = -11286.7;
    p1.y = -2602.99;
    p2.x = -11286.6;
    p2.y = -2602.95;
    p3.x = -11250.6;
    p3.y = -2583.73;
    p4.x = -11166.6;
    p4.y = -2543.2;
    flag = Tools::isLineSegmentInteracted(p1, p2, p3, p4);
    EXPECT_FALSE(flag) << "line segment2 interact test";
}

// 矩形相交测试
TEST_F(MotionPlanningUnitTests, rectInteractTest) {
    // 初始化两个矩形
    Rectangle rect1, rect2;
    rect1.center_x_ = 0.0;
    rect1.center_y_ = 0.0;
    rect1.width_ = 3.42;
    rect1.length_ = 4.64356;
    rect1.rotation_ = 0.0;
    rect2.center_x_ = 1.0;
    rect2.center_y_ = 2.0;
    rect2.width_ = 0.01;
    rect2.length_ = 5.6925;
    rect2.rotation_ = 0.623883;

    bool result;
    result = Tools::isRectangleOverlap(rect1, rect2, 1.0, 1.0);
    EXPECT_TRUE(result) << "rectangle1 interact test";
    
    rect1.center_x_ = -11333.8;
    rect1.center_y_ = -2621.72;
    rect1.width_ = 3.42;
    rect1.length_ = 4.64356;
    rect1.rotation_ = 0.449365;
    rect2.center_x_ = -11336.7;
    rect2.center_y_ = -2624.9;
    rect2.width_ = 2.1436;
    rect2.length_ = 5.6925;
    rect2.rotation_ = 0.623883;
    result = Tools::isRectangleOverlap(rect1, rect2, 1.0, 1.0);
    EXPECT_TRUE(result) << "rectangle2 interact test";
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}