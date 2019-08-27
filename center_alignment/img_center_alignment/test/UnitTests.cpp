/*
 * UnitTests.cpp
 * Copyright (C) 2019 theo <theo@not-arch-linux>
 *
 * Distributed under terms of the MIT license.
 */

#include <time.h>
#include <stdlib.h>
#include <math.h>
#include <gtest/gtest.h>
#include "img_center_alignment/Controller.h"


#define rand_a_b(a, b) ((float)rand()/(float)RAND_MAX)*(b-a)+a


Controller* make_controller(State state)
{
   gain_param k_x, k_y;
   k_x.insert(std::pair<std::string, float>("p", 0.5));
   k_x.insert(std::pair<std::string, float>("i", 0.1));
   k_x.insert(std::pair<std::string, float>("d", 0.2));

   k_y.insert(std::pair<std::string, float>("p", 0.2));
   k_y.insert(std::pair<std::string, float>("i", 0.0));
   k_y.insert(std::pair<std::string, float>("d", 0.0));

   Controller *controller = new Controller(k_x, k_y, 0.25);
   controller->SetState(state);

   return controller;
}


void make_prediction(Prediction *p, int xMax, int xMin, int yMax, int yMin,
      bool locked)
{
   p->bbox = perception::Bbox();
   p->bbox.minX = xMin;
   p->bbox.minY = yMin;
   p->bbox.maxX = xMax;
   p->bbox.maxY = yMax;
   p->locked = locked;
   p->x = (xMax + xMin) / 2;
   p->y = (yMax + yMin) / 2;
}


void make_random_prediction(Prediction *p)
{
   float ratio;
   do { 
      p->bbox = perception::Bbox();
      p->bbox.maxX = rand_a_b(50, IMG_WIDTH);
      p->bbox.minX = rand_a_b(0, p->bbox.maxX);
      p->bbox.maxY = rand_a_b(50, IMG_HEIGHT);
      p->bbox.minY = rand_a_b(0, p->bbox.maxY);
      p->x = (p->bbox.maxX + p->bbox.minX) / 2;
      p->y = (p->bbox.maxY + p->bbox.minY) / 2;
      ratio =
         (float)(p->bbox.maxX-p->bbox.minX)/(float)(p->bbox.maxY-p->bbox.minY);
   } while (ratio == 0);
}


TEST(Controller, Init) {
   Controller *controller = make_controller(CALIBRATING);
   Prediction center;
   int width, height;
   float ratio;
   make_prediction(&center, 100, 45, 235, 119, true);
   width = center.bbox.maxX - center.bbox.minX;
   height = center.bbox.maxY - center.bbox.minY;
   ratio = (float)width/(float)height;

   auto refGate = controller->GetRefGate();
   EXPECT_EQ(refGate.ratio, 0);
   EXPECT_EQ(refGate.width, 0);
   EXPECT_EQ(refGate.height, 0);

   controller->SetGateCenter(center);
   controller->Step(0);

   refGate = controller->GetRefGate();
   EXPECT_EQ(refGate.ratio, 0);
   EXPECT_EQ(refGate.width, 0);
   EXPECT_EQ(refGate.height, 0);

   auto buffer = controller->GetRefGateBuffer();
   EXPECT_EQ(buffer.front().ratio, ratio);
   EXPECT_EQ(buffer.front().width, width);
   EXPECT_EQ(buffer.front().height, height);

   delete controller;
}

TEST(Controller, InitMultiple) {
   Controller *controller = make_controller(CALIBRATING);
   Prediction center;
   int width, height;
   float ratio;
   make_prediction(&center, 100, 45, 235, 119, true);
   width = center.bbox.maxX - center.bbox.minX;
   height = center.bbox.maxY - center.bbox.minY;
   ratio = (float)width/(float)height;

   for (int i = 0; i < CALIBRATION_QUEUE_SIZE + 5; ++i) {
      auto refGate = controller->GetRefGate();
      EXPECT_EQ(refGate.ratio, 0);
      EXPECT_EQ(refGate.width, 0);
      EXPECT_EQ(refGate.height, 0);

      controller->SetGateCenter(center);
      controller->Step(0);

      refGate = controller->GetRefGate();
      EXPECT_EQ(refGate.ratio, 0);
      EXPECT_EQ(refGate.width, 0);
      EXPECT_EQ(refGate.height, 0);

      auto buffer = controller->GetRefGateBuffer();
      EXPECT_EQ(buffer.front().ratio, ratio);
      EXPECT_EQ(buffer.front().width, width);
      EXPECT_EQ(buffer.front().height, height);

      controller->SetState(AIMING);
      controller->SetGateCenter(center);
      controller->Step(0);
      controller->Step(0);
   }

   delete controller;
}


TEST(Controller, CompleteCalibrationIdeal) {
   Controller *controller = make_controller(CALIBRATING);
   Prediction center;
   int width, height, meanWidth, meanHeight;
   float meanRatio;
   make_prediction(&center, 100, 45, 235, 119, true);
   width = center.bbox.maxX - center.bbox.minX;
   height = center.bbox.maxY - center.bbox.minY;
   meanWidth = meanHeight = meanRatio = 0;

   /* Publish 5 valid predictions and observe the ref_gate to be set */
   for (int i = 0; i < CALIBRATION_QUEUE_SIZE; ++i) {
      controller->SetGateCenter(center);
      controller->Step(0);
      meanWidth += width;
      meanHeight += height;
   }

   // TODO: Use the median instead!

   meanWidth /= CALIBRATION_QUEUE_SIZE;
   meanHeight /= CALIBRATION_QUEUE_SIZE;
   meanRatio = (float)meanWidth/(float)meanHeight;

   auto refGate = controller->GetRefGate();
   EXPECT_EQ(controller->GetState(), FLYING);
   EXPECT_EQ(refGate.ratio, meanRatio);
   EXPECT_EQ(refGate.width, meanWidth);
   EXPECT_EQ(refGate.height, meanHeight);
}


TEST(Controller, CompleteCalibrationWithControlledVariance) {
   Controller *controller = make_controller(CALIBRATING);
   Prediction centers[CALIBRATION_QUEUE_SIZE];
   make_prediction(&centers[0], 140, 85, 187, 112, true);
   make_prediction(&centers[1], 137, 94, 177, 119, true);
   make_prediction(&centers[2], 131, 78, 192, 106, true);
   make_prediction(&centers[3], 144, 81, 183, 118, true);
   make_prediction(&centers[4], 146, 89, 180, 110, true);
   make_prediction(&centers[5], 143, 82, 185, 109, true);
   make_prediction(&centers[6], 135, 97, 179, 120, true);
   make_prediction(&centers[7], 137, 82, 186, 111, true);
   make_prediction(&centers[8], 141, 79, 185, 117, true);
   make_prediction(&centers[9], 145, 90, 182, 113, true);
   int width, height, meanWidth, meanHeight;
   float meanRatio;
   meanWidth = meanHeight = meanRatio = 0;

   for (int i = 0; i < CALIBRATION_QUEUE_SIZE; ++i) {
      width = centers[i].bbox.maxX - centers[i].bbox.minX;
      height = centers[i].bbox.maxY - centers[i].bbox.minY;
      controller->SetGateCenter(centers[i]);
      controller->Step(0);
      meanWidth += width;
      meanHeight += height;
   }

   meanWidth /= CALIBRATION_QUEUE_SIZE;
   meanHeight /= CALIBRATION_QUEUE_SIZE;
   meanRatio = (float)meanWidth/(float)meanHeight;

   auto refGate = controller->GetRefGate();
   EXPECT_EQ(controller->GetState(), FLYING);
   EXPECT_EQ(refGate.ratio, meanRatio);
   EXPECT_EQ(refGate.width, meanWidth);
   EXPECT_EQ(refGate.height, meanHeight);
}


TEST(Controller, CompleteCalibrationWithControlledVarianceInvalid) {
   Controller *controller = make_controller(CALIBRATING);
   Prediction centers[CALIBRATION_QUEUE_SIZE];
   make_prediction(&centers[0], 140, 85, 187, 112, true);
   make_prediction(&centers[1], 137, 94, 177, 119, true);
   make_prediction(&centers[2], 131, 78, 192, 106, true);
   make_prediction(&centers[3], 144, 81, 183, 118, true);
   make_prediction(&centers[4], 146, 89, 180, 110, false);
   make_prediction(&centers[5], 143, 82, 185, 109, true);
   make_prediction(&centers[6], 135, 97, 179, 120, true);
   make_prediction(&centers[7], 137, 82, 186, 111, true);
   make_prediction(&centers[8], 141, 79, 185, 117, true);
   make_prediction(&centers[9], 145, 90, 182, 113, true);
   int width, height, meanWidth, meanHeight;
   float meanRatio;
   meanWidth = meanHeight = meanRatio = 0;

   for (int i = 0; i < CALIBRATION_QUEUE_SIZE; ++i) {
      width = centers[i].bbox.maxX - centers[i].bbox.minX;
      height = centers[i].bbox.maxY - centers[i].bbox.minY;
      controller->SetGateCenter(centers[i]);
      controller->Step(0);
      meanWidth += width;
      meanHeight += height;
   }

   meanWidth /= CALIBRATION_QUEUE_SIZE;
   meanHeight /= CALIBRATION_QUEUE_SIZE;
   meanRatio = (float)meanWidth/(float)meanHeight;

   auto refGate = controller->GetRefGate();
   EXPECT_EQ(controller->GetState(), CALIBRATING);
   EXPECT_NE(refGate.ratio, meanRatio);
   EXPECT_NE(refGate.width, meanWidth);
   EXPECT_NE(refGate.height, meanHeight);
}


TEST(Controller, CompleteCalibrationWithControlledVarianceInvalidThenValid) {
   Controller *controller = make_controller(CALIBRATING);
   Prediction centers[CALIBRATION_QUEUE_SIZE];
   make_prediction(&centers[0], 140, 85, 187, 112, true);
   make_prediction(&centers[1], 137, 94, 177, 119, true);
   make_prediction(&centers[2], 131, 78, 192, 106, true);
   make_prediction(&centers[3], 144, 81, 183, 118, true);
   make_prediction(&centers[4], 146, 89, 180, 110, true);
   make_prediction(&centers[5], 143, 82, 185, 109, true);
   make_prediction(&centers[6], 135, 97, 179, 120, true);
   make_prediction(&centers[7], 13, 25, 86, 199, true); // Invalid!
   make_prediction(&centers[8], 141, 79, 185, 117, true);
   make_prediction(&centers[9], 145, 90, 182, 113, true);
   int width, height, meanWidth, meanHeight;
   float meanRatio;
   meanWidth = meanHeight = meanRatio = 0;

   for (int i = 0; i < CALIBRATION_QUEUE_SIZE; ++i) {
      controller->SetGateCenter(centers[i]);
      controller->Step(0);
      meanWidth += centers[i].bbox.maxX - centers[i].bbox.minX;
      meanHeight += centers[i].bbox.maxY - centers[i].bbox.minY;
   }

   meanWidth /= CALIBRATION_QUEUE_SIZE;
   meanHeight /= CALIBRATION_QUEUE_SIZE;
   meanRatio = (float)meanWidth/(float)meanHeight;

   auto refGate = controller->GetRefGate();
   EXPECT_EQ(controller->GetState(), CALIBRATING);
   /* Values should be reset */
   EXPECT_EQ(refGate.width, 0);
   EXPECT_EQ(refGate.height, 0);
   EXPECT_EQ(refGate.ratio, 0);

   Prediction validCenter;
   make_prediction(&validCenter, 142, 90, 181, 111, true);
   centers[7] = validCenter; // Swap the invalid one for the new valid center

   meanWidth = meanHeight = meanRatio = 0;
   for (int i = 0; i < CALIBRATION_QUEUE_SIZE; ++i) {
      controller->SetGateCenter(centers[i]);
      controller->Step(0);
      meanWidth += centers[i].bbox.maxX - centers[i].bbox.minX;
      meanHeight += centers[i].bbox.maxY - centers[i].bbox.minY;
   }
   meanWidth /= CALIBRATION_QUEUE_SIZE;
   meanHeight /= CALIBRATION_QUEUE_SIZE;
   meanRatio = (float)meanWidth/(float)meanHeight;
   refGate = controller->GetRefGate();

   EXPECT_EQ(controller->GetState(), FLYING);
   EXPECT_EQ(refGate.ratio, meanRatio);
   EXPECT_EQ(refGate.width, meanWidth);
   EXPECT_EQ(refGate.height, meanHeight);
}
/*
TEST(Controller, CompleteCalibrationWithVariance) {
   Controller *controller = make_controller(CALIBRATING);
   Prediction center;
   int width, height, meanWidth, meanHeight;
   float meanRatio;
   meanWidth = meanHeight = meanRatio = 0;

   while (controller->GetRefGateBuffer().size() < CALIBRATION_QUEUE_SIZE) {
      make_random_prediction(&center);
      width = center.bbox.maxX - center.bbox.minX;
      height = center.bbox.maxY - center.bbox.minY;
      std::cout << width << " x " << height << std::endl;
      controller->SetGateCenter(center);
      controller->Step(0);
      meanWidth += width;
      meanHeight += height;
   }

   meanWidth /= CALIBRATION_QUEUE_SIZE;
   meanHeight /= CALIBRATION_QUEUE_SIZE;
   meanRatio = (float)meanWidth/(float)meanHeight;

   auto refGate = controller->GetRefGate();
   EXPECT_EQ(controller->GetState(), FLYING);
   EXPECT_EQ(refGate.ratio, meanRatio);
   EXPECT_EQ(refGate.width, meanWidth);
   EXPECT_EQ(refGate.height, meanHeight);
}
*/
int main(int argc, char **argv) {
   srand((unsigned) time(NULL));
   testing::InitGoogleTest(&argc, argv);
   ros::init(argc, argv, "tester");
   ros::NodeHandle nh;

   return RUN_ALL_TESTS();
}
