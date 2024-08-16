/*
 * @Author: Tommy0929
 * @Date: 2024-01-24 22:35:20
 * @LastEditors: Tommy0929 tommy07210728@163.com
 * @FilePath: /final/inc/main.h
 * WHUROBOCON_SAVED!!!
 */
#pragma once
#include <pthread.h>
#include "uart.h"
#include "camera.h"
#include "opencv.h"
#include "detect.h"
#include "pointcloud.h"

pthread_mutex_t protect_picture;
pthread_mutex_t buff_mutex;
pthread_mutex_t infer_mutex;

cv::Mat rgb_frame, depth_frame, *rgb_ptr, *rgb_ptr2, *rgb_ptr3, *depth_ptr;

k4a::capture capture;
k4a::transformation k4aTransformation;
k4a::calibration k4aCalibration;
k4a::capture Capture;
k4a::transformation K4aTransformation;
k4a::calibration K4aCalibration;

std::shared_future<Yolo::BoxArray> prefuture;
std::shared_ptr<cv::Mat> matBuff;
std::shared_ptr<cv::Mat> matBuff2;
std::shared_ptr<cv::Mat> matBuff3;
std::shared_ptr<cv::Mat> depthBuff;
std::shared_ptr<cv::Mat> depthBuff2;

camera *camera_ptr;
detect *Detect;

class pthread
{
public:
    void *k4aUpdate(void *camera_ptr);
    void *picture(void *argc);
    void *picture2(void *argc);
    void *create_infer(void *argc);
    void *infer_labels(void *argc);

private:
    camera *camera_ptr;
};

void *k4aUpdate(void *num)
{   

    camera *Camera = camera_ptr;

    int *index = (int *)num;

    k4a::device Device = Camera->init_kinect(capture, k4aTransformation, k4aCalibration, *index);

    while (1)
    {

        pthread_mutex_lock(&protect_picture);

        Camera->picture_update(Device, capture);

        pthread_mutex_lock(&buff_mutex);

        rgb_ptr = Camera->getpicture(Device, capture, rgb_frame, k4aTransformation);
        rgb_ptr2 = Camera->getpicture(Device, capture, rgb_frame, k4aTransformation);
        depth_ptr = Camera->getdepth(Device, capture, depth_frame, k4aTransformation);

        Camera->getAngel(Device);

        pthread_mutex_unlock(&buff_mutex);

        pthread_mutex_lock(&infer_mutex);
        cv::cvtColor(*rgb_ptr, *rgb_ptr, cv::COLOR_BGR2HSV);
        cv::cvtColor(*rgb_ptr, *rgb_ptr, cv::COLOR_HSV2BGR);
        cv::cvtColor(*rgb_ptr2, *rgb_ptr2, cv::COLOR_BGR2HSV);
        cv::cvtColor(*rgb_ptr2, *rgb_ptr2, cv::COLOR_HSV2BGR);
        pthread_mutex_unlock(&infer_mutex);

        pthread_mutex_lock(&buff_mutex);

        matBuff = std::make_shared<cv::Mat>(rgb_ptr->clone());
        matBuff2 = std::make_shared<cv::Mat>(rgb_ptr2->clone());
        depthBuff = std::make_shared<cv::Mat>(depth_ptr->clone());
        depthBuff2 = std::make_shared<cv::Mat>(depth_ptr->clone());

        pthread_mutex_unlock(&buff_mutex);

        pthread_mutex_unlock(&protect_picture);

        usleep(10000);
cv::imshow("test",*matBuff);
cv::waitKey(1);
        if (matBuff->empty() || matBuff2->empty() || depthBuff->empty())
        {
            cout << "error" << endl;
        }

        rgb_ptr->release();
        rgb_ptr2->release();
        depth_ptr->release();
    }
    pthread_exit(NULL);
}

void *k4aUpdate2(void *num)
{

    camera *Camera = camera_ptr;

    int *index = (int *)num;

    k4a::device device = Camera->init_kinect(Capture, K4aTransformation, K4aCalibration, *index);

    while (1)
    {

        pthread_mutex_lock(&protect_picture);

        Camera->picture_update(device, Capture);

        pthread_mutex_lock(&buff_mutex);

        rgb_ptr3 = Camera->getpicture(device, Capture, rgb_frame, K4aTransformation);
        
        Camera->getAngel(device);

        pthread_mutex_unlock(&buff_mutex);

        pthread_mutex_lock(&infer_mutex);

        cv::cvtColor(*rgb_ptr3, *rgb_ptr3, cv::COLOR_BGR2HSV);
        cv::cvtColor(*rgb_ptr3, *rgb_ptr3, cv::COLOR_HSV2BGR);

        pthread_mutex_unlock(&infer_mutex);

        pthread_mutex_lock(&buff_mutex);

        matBuff3 = std::make_shared<cv::Mat>(rgb_ptr3->clone());

        pthread_mutex_unlock(&buff_mutex);

        pthread_mutex_unlock(&protect_picture);

        usleep(10000);

        if (matBuff3->empty())
        {
            cout << "error" << endl;
        }

        rgb_ptr3->release();
    }
    pthread_exit(NULL);
}

void *create_infer(void *argc)
{
    auto yoloEngine = Yolo::create_infer("red.trtmodel", Yolo::Type::V5, 0, 0.8f, 0.5f);
    
    while (1)
    {
        if (prefuture.valid())
        {
            pthread_mutex_lock(&protect_picture);
            pthread_mutex_lock(&buff_mutex);

            prefuture = yoloEngine->commit(*matBuff2);
            auto bboxes = prefuture.get();

            pthread_mutex_unlock(&buff_mutex);
            usleep(10000);

            int color_num[7] = {156, 43, 46, 180, 255, 255, 0x02};

            uint32_t NUM = bboxes.size();

            pthread_mutex_lock(&buff_mutex);

            for (auto box : bboxes)
            {
                Detect->detect_boxes(bboxes, *matBuff2, NUM, *depthBuff, k4aTransformation, k4aCalibration, color_num);
                // cout << "Red success" << endl;
            }

            pthread_mutex_unlock(&buff_mutex);
            usleep(10000);
        }
        else
        {
            prefuture = yoloEngine->commit(*matBuff2);
        }

        pthread_mutex_lock(&buff_mutex);

        cv::imshow("red", *matBuff2);
        cv::waitKey(1);

        pthread_mutex_unlock(&buff_mutex);
        pthread_mutex_unlock(&protect_picture);

        usleep(10000);
    }
    pthread_exit(NULL);
}

void *picture(void *argc)
{

    auto yoloEngine = Yolo::create_infer("purple.trtmodel", Yolo::Type::V5, 0, 0.8f, 0.5f);

    while (1)
    {

        if (prefuture.valid())
        {
            pthread_mutex_lock(&infer_mutex);

            pthread_mutex_lock(&buff_mutex);

            auto bboxes = yoloEngine->commit(*matBuff).get();

            pthread_mutex_unlock(&buff_mutex);
            usleep(1000);

            int color_num[7] = {156, 43, 46, 180, 255, 255, 0x02};

            uint32_t NUM = bboxes.size();

            Detect->detect_distance_no_threead(bboxes, *matBuff, NUM);
        }
        else
        {
            prefuture = yoloEngine->commit(*matBuff);
        }

        pthread_mutex_lock(&buff_mutex);

        cv::imshow("purple", *matBuff);
        cv::waitKey(1);

        pthread_mutex_unlock(&buff_mutex);
        pthread_mutex_unlock(&infer_mutex);

        usleep(1000);
    }

    pthread_exit(NULL);
}

void *picture2(void *argc)
{
    auto yoloEngine = Yolo::create_infer("purple.trtmodel", Yolo::Type::V5, 0, 0.8f, 0.5f);

    while (1)
    {

        if (prefuture.valid())
        {
            pthread_mutex_lock(&protect_picture);
            pthread_mutex_lock(&buff_mutex);

            auto bboxes = yoloEngine->commit(*matBuff).get();

            pthread_mutex_unlock(&buff_mutex);
            usleep(10000);

            int color_num[7] = {125, 43, 46, 155, 255, 255, 0x03};

            uint32_t NUM = bboxes.size();

            pthread_mutex_lock(&buff_mutex);

            // ObjectDetector::Box boxes[NUM];

            for (auto box : bboxes)
            {
                Detect->detect_boxes(bboxes, *matBuff, NUM, *depthBuff2, k4aTransformation, k4aCalibration, color_num);
                // cout << "Purple success" << endl;
            }

            pthread_mutex_unlock(&buff_mutex);
            usleep(10000);
        }
        else
        {
            prefuture = yoloEngine->commit(*matBuff);
        }

        pthread_mutex_lock(&buff_mutex);

        cv::imshow("purple", *matBuff);
        cv::waitKey(1);

        pthread_mutex_unlock(&buff_mutex);
        pthread_mutex_unlock(&protect_picture);

        usleep(10000);
    }

    pthread_exit(NULL);
}

void *infer_labels(void *argc)
{
    auto yoloEngine = Yolo::create_infer("twolabels.trtmodel", Yolo::Type::V5, 0, 0.8f, 0.5f);

    while (1)
    {

        if (prefuture.valid())
        {

            pthread_mutex_lock(&protect_picture);
            pthread_mutex_lock(&buff_mutex);

            prefuture = yoloEngine->commit(*matBuff3);
            auto bboxes = prefuture.get();

            pthread_mutex_unlock(&buff_mutex);
            usleep(1000);

            uint32_t NUM = bboxes.size();

            pthread_mutex_lock(&buff_mutex);

            for (auto box : bboxes)
            {
                Detect->detect_labels(bboxes, *matBuff3, NUM);
            }

            pthread_mutex_unlock(&buff_mutex);
            usleep(1000);
        }
        else
        {
            prefuture = yoloEngine->commit(*matBuff3);
        }

        pthread_mutex_lock(&buff_mutex);

        cv::imshow("labels", *matBuff3);
        cv::waitKey(1);

        pthread_mutex_unlock(&buff_mutex);
        pthread_mutex_unlock(&protect_picture);

        usleep(1000);
    }
    pthread_exit(NULL);
}
