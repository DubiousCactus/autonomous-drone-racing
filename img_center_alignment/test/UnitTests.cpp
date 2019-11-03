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


perception::Bbox make_bbox(int xMax, int xMin, int yMax, int yMin)
{
   perception::Bbox bbox;
   bbox.minX = xMin;
   bbox.minY = yMin;
   bbox.maxX = xMax;
   bbox.maxY = yMax;

   return bbox;
}


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

   if (state == FLYING) {
      controller->SetState(CALIBRATING);
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

      for (int i = 0; i < CALIBRATION_QUEUE_SIZE; ++i) {
         controller->SetGateCenter(centers[i]);
         controller->Step();
      }
   }
   controller->SetState(state);

   return controller;
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
   controller->Step();

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
      controller->Step();

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
      controller->Step();
      controller->Step();
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
      controller->Step();
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
      controller->SetGateCenter(centers[i]);
      controller->Step();
      meanWidth += centers[i].bbox.maxX - centers[i].bbox.minX;
      meanHeight += centers[i].bbox.maxY - centers[i].bbox.minY;
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
      controller->SetGateCenter(centers[i]);
      controller->Step();
      meanWidth += centers[i].bbox.maxX - centers[i].bbox.minX;
      meanHeight += centers[i].bbox.maxY - centers[i].bbox.minY;
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
      controller->Step();
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
      controller->Step();
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


TEST(Controller, CalibrationAndFlight) {
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
      controller->SetGateCenter(centers[i]);
      controller->Step();
      meanWidth += centers[i].bbox.maxX - centers[i].bbox.minX;
      meanHeight += centers[i].bbox.maxY - centers[i].bbox.minY;
   }

   meanWidth /= CALIBRATION_QUEUE_SIZE;
   meanHeight /= CALIBRATION_QUEUE_SIZE;
   meanRatio = (float)meanWidth/(float)meanHeight;

   auto refGate = controller->GetRefGate();
   EXPECT_EQ(controller->GetState(), FLYING);
   EXPECT_EQ(refGate.ratio, meanRatio);
   EXPECT_EQ(refGate.width, meanWidth);
   EXPECT_EQ(refGate.height, meanHeight);

   Prediction validCenter;
   make_prediction(&validCenter, 138, 84, 177, 113, true);
   controller->SetGateCenter(validCenter);
   controller->Step();

   Vector3d true_err = Vector3d(CAM_WIDTH/2, CAM_HEIGHT/2) -
      Vector3d(validCenter.x, validCenter.y);
   auto alignmentError = controller->GetAlignmentError();
   EXPECT_EQ(true_err.x, alignmentError.x);
   EXPECT_EQ(true_err.y, alignmentError.y);
}

TEST(Controller, CalibrationAndFlightWithCompensation) {
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
      controller->SetGateCenter(centers[i]);
      controller->Step();
      meanWidth += centers[i].bbox.maxX - centers[i].bbox.minX;
      meanHeight += centers[i].bbox.maxY - centers[i].bbox.minY;
   }

   meanWidth /= CALIBRATION_QUEUE_SIZE;
   meanHeight /= CALIBRATION_QUEUE_SIZE;
   meanRatio = (float)meanWidth/(float)meanHeight;

   auto refGate = controller->GetRefGate();
   EXPECT_EQ(controller->GetState(), FLYING);
   EXPECT_EQ(refGate.ratio, meanRatio);
   EXPECT_EQ(refGate.width, meanWidth);
   EXPECT_EQ(refGate.height, meanHeight);

   Prediction invalidCenter;
   make_prediction(&invalidCenter, 132, 83, 200, 174, true); // Bad ratio
   controller->SetGateCenter(invalidCenter);
   controller->Step();

   int correctedHeight = (invalidCenter.bbox.maxX-invalidCenter.bbox.minX)/meanRatio;
   Vector3d correctedCenter = Vector3d(invalidCenter.x, invalidCenter.y);
   correctedCenter.y = (invalidCenter.y > CAM_HEIGHT/2) ?
      (invalidCenter.bbox.minY + correctedHeight + invalidCenter.bbox.maxY)/2 :
      (invalidCenter.bbox.maxY - correctedHeight + invalidCenter.bbox.minY)/2;
   Vector3d corrected_err = Vector3d(CAM_WIDTH/2, CAM_HEIGHT/2) - correctedCenter;
   auto alignmentError = controller->GetAlignmentError();
   EXPECT_EQ(corrected_err.x, alignmentError.x);
   EXPECT_EQ(corrected_err.y, alignmentError.y);

   make_prediction(&invalidCenter, 132, 105, 200, 136, true); // Bad ratio
   controller->SetGateCenter(invalidCenter);
   controller->Step();

   int correctedWidth = (invalidCenter.bbox.maxY-invalidCenter.bbox.minY)*meanRatio;
   correctedCenter = Vector3d(invalidCenter.x, invalidCenter.y);
   correctedCenter.x = (invalidCenter.x > CAM_WIDTH/2) ?
      (invalidCenter.bbox.minX + correctedWidth + invalidCenter.bbox.maxX)/2 :
      (invalidCenter.bbox.maxX - correctedWidth + invalidCenter.bbox.minX)/2;
   corrected_err = Vector3d(CAM_WIDTH/2, CAM_HEIGHT/2) - correctedCenter;
   alignmentError = controller->GetAlignmentError();
   EXPECT_EQ(corrected_err.x, alignmentError.x);
   EXPECT_EQ(corrected_err.y, alignmentError.y);
}


TEST(Controller, CrossingCondition) {
   Controller *controller = make_controller(FLYING);
   // PREVIOUD_PREDICTIONS_CNT = 10
   std::list<perception::Bbox> prevPreds;
   bool crossing;
   Prediction vanishedCenter;
   
   prevPreds = {
      make_bbox(140, 101, 167, 88),
      make_bbox(163, 98, 172, 73),
      make_bbox(169, 92, 179, 68),
      make_bbox(174, 86, 187, 61),
      make_bbox(188, 75, 198, 50),
      make_bbox(199, 61, 210, 43),
      make_bbox(209, 56, 218, 34),
      make_bbox(233, 50, 224, 29),
      make_bbox(259, 25, 218, 16),
      make_bbox(302, 12, 223, 8)
   }; 
   controller->SetPreviousPredictions(prevPreds);
   make_prediction(&vanishedCenter, 0, 0, 0, 0, false);
   controller->SetGateCenter(vanishedCenter);
   controller->Step();

   crossing = controller->CrossingCondition();
   EXPECT_TRUE(crossing);
   
   prevPreds = {
      make_bbox(140, 101, 167, 88),
      make_bbox(140, 101, 167, 88),
      make_bbox(140, 101, 167, 88),
      make_bbox(140, 101, 167, 88),
      make_bbox(140, 101, 167, 88),
      make_bbox(140, 101, 167, 88),
      make_bbox(140, 101, 167, 88),
      make_bbox(140, 101, 167, 88),
      make_bbox(140, 101, 167, 88),
      make_bbox(140, 101, 167, 88)
   }; 
   controller->SetPreviousPredictions(prevPreds);
   make_prediction(&vanishedCenter, 0, 0, 0, 0, false);
   controller->SetGateCenter(vanishedCenter);
   controller->Step();

   crossing = controller->CrossingCondition();
   EXPECT_FALSE(crossing);

   prevPreds = {
      make_bbox(140, 101, 167, 88),
      make_bbox(163, 98, 172, 73),
      make_bbox(163, 98, 172, 73),
      make_bbox(174, 86, 187, 61),
      make_bbox(188, 75, 198, 50),
      make_bbox(188, 75, 198, 50),
      make_bbox(209, 56, 218, 34),
      make_bbox(233, 50, 224, 29),
      make_bbox(233, 50, 224, 29),
      make_bbox(302, 12, 223, 8)
   }; 
   controller->SetPreviousPredictions(prevPreds);
   make_prediction(&vanishedCenter, 0, 0, 0, 0, false);
   controller->SetGateCenter(vanishedCenter);
   controller->Step();

   crossing = controller->CrossingCondition();
   EXPECT_TRUE(crossing);

   prevPreds = {
      make_bbox(140, 101, 167, 88),
      make_bbox(163, 98, 172, 73),
      make_bbox(169, 92, 179, 68),
      make_bbox(174, 86, 187, 61),
      make_bbox(188, 75, 198, 50),
      make_bbox(199, 61, 210, 43),
      make_bbox(209, 56, 218, 34),
      make_bbox(233, 50, 224, 29),
      make_bbox(259, 25, 218, 16),
      make_bbox(302, 12, 223, 8)
   }; 
   controller->SetPreviousPredictions(prevPreds);
   make_prediction(&vanishedCenter, 0, 0, 0, 0, true);
   controller->SetGateCenter(vanishedCenter);
   controller->Step();

   crossing = controller->CrossingCondition();
   EXPECT_FALSE(crossing);
   
   prevPreds = {
      make_bbox(140, 101, 167, 88),
      make_bbox(163, 98, 172, 73),
      make_bbox(169, 92, 179, 68),
      make_bbox(174, 86, 187, 61),
      make_bbox(172, 110, 168, 80),
      make_bbox(199, 61, 210, 43),
      make_bbox(209, 56, 218, 34),
      make_bbox(233, 50, 224, 29),
      make_bbox(259, 25, 218, 16),
      make_bbox(302, 12, 223, 8)
   }; 
   controller->SetPreviousPredictions(prevPreds);
   make_prediction(&vanishedCenter, 0, 0, 0, 0, false);
   controller->SetGateCenter(vanishedCenter);
   controller->Step();

   crossing = controller->CrossingCondition();
   EXPECT_FALSE(crossing);
 
   prevPreds = {
      make_bbox(140, 101, 167, 88),
      make_bbox(163, 98, 172, 73),
      make_bbox(169, 92, 179, 68),
      make_bbox(174, 86, 187, 61),
      make_bbox(172, 110, 168, 80),
      make_bbox(199, 61, 210, 43),
      make_bbox(209, 56, 218, 34),
      make_bbox(233, 50, 224, 29),
      make_bbox(259, 25, 218, 16),
      make_bbox(302, 12, 223, 8)
   }; 
   controller->SetPreviousPredictions(prevPreds);
   make_prediction(&vanishedCenter, 0, 0, 0, 0, true);
   controller->SetGateCenter(vanishedCenter);
   controller->Step();

   crossing = controller->CrossingCondition();
   EXPECT_FALSE(crossing);
}


int main(int argc, char **argv) {
   srand((unsigned) time(NULL));
   testing::InitGoogleTest(&argc, argv);
   ros::init(argc, argv, "tester");
   ros::NodeHandle nh;

   return RUN_ALL_TESTS();
}
