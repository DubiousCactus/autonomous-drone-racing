/*
 * UnitTests.cpp
 * Copyright (C) 2019 theo <theo@not-arch-linux>
 *
 * Distributed under terms of the MIT license.
 */

#include <gtest/gtest.h>
#include "img_center_alignment/Controller.h"


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


TEST(Controller, Init) {
   Controller *controller = make_controller(CALIBRATING);
   int width, height;
   float ratio;
   perception::Bbox bbox;
   Prediction center;
   bbox = perception::Bbox();
   bbox.minX = 45;
   bbox.maxX = 100;
   bbox.minY = 119;
   bbox.maxY = 235;
   center = { bbox, true, 140, 220 };
   width = bbox.maxX - bbox.minX;
   height = bbox.maxY - bbox.minY;
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
   int width, height;
   float ratio;
   perception::Bbox bbox;
   Prediction center;
   bbox = perception::Bbox();
   bbox.minX = 45;
   bbox.maxX = 100;
   bbox.minY = 119;
   bbox.maxY = 235;
   center = { bbox, true, 140, 220 };
   width = bbox.maxX - bbox.minX;
   height = bbox.maxY - bbox.minY;
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
   int width, height, meanWidth, meanHeight;
   float ratio, meanRatio;
   perception::Bbox bbox;
   Prediction center;
   bbox = perception::Bbox();
   bbox.minX = 45;
   bbox.maxX = 100;
   bbox.minY = 119;
   bbox.maxY = 235;
   center = { bbox, true, 140, 220 };
   width = bbox.maxX - bbox.minX;
   height = bbox.maxY - bbox.minY;
   ratio = (float)width/(float)height;
   meanWidth = meanHeight = meanRatio = 0;

   /* Publish 5 valid predictions and observe the ref_gate to be set */
   for (int i = 0; i < CALIBRATION_QUEUE_SIZE; ++i) {
      controller->SetGateCenter(center);
      controller->Step(0);
      meanWidth += width;
      meanHeight += height;
      meanRatio += ratio;
   }

   meanWidth /= CALIBRATION_QUEUE_SIZE;
   meanHeight /= CALIBRATION_QUEUE_SIZE;
   meanRatio /= CALIBRATION_QUEUE_SIZE;

   controller->Step(0);
   auto refGate = controller->GetRefGate();
   EXPECT_EQ(controller->GetState(), FLYING);
   EXPECT_EQ(refGate.ratio, meanRatio);
   EXPECT_EQ(refGate.width, meanWidth);
   EXPECT_EQ(refGate.height, meanHeight);
}


int main(int argc, char **argv) {
   testing::InitGoogleTest(&argc, argv);
   ros::init(argc, argv, "tester");
   ros::NodeHandle nh;

   return RUN_ALL_TESTS();
}
