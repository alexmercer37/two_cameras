/*
 * @Author: ddxy
 * @Date: 2023-10-10 10:43:39
 * @LastEditors: Tommy0929 tommy07210728@163.com
 * @FilePath: /final/src/main.cpp
 * WHUROBOCON_SAVED!!!
 */ \
#include "../inc/main.h"

int main(int argc, char const *argv[])
{
 //     TRT::compile(
 //     TRT::Mode::FP16,
 //     1,
 //     "/home/nf/Downloads/twocamera/final/workspace/red.onnx",
 //     "red.trtmodel");
 //    cout << "Done" << endl;
 
    uart_init();

    camera_ptr = new camera;

    pthread_mutex_init(&protect_picture, nullptr);
    pthread_mutex_init(&buff_mutex, nullptr);
    pthread_mutex_init(&infer_mutex, nullptr);

    matBuff.reset(new cv::Mat);
    matBuff2.reset(new cv::Mat);
    matBuff3.reset(new cv::Mat);
    depthBuff.reset(new cv::Mat);
    depthBuff2.reset(new cv::Mat);

    pthread_t threads[10] = {0};

    int index1 = 0;
    int index2 = 1;

    pthread_create(&threads[0], NULL, k4aUpdate, (void *)&index1);
  //  pthread_create(&threads[1], NULL, k4aUpdate2, (void *)&index2);
    sleep(2);
    pthread_create(&threads[2], NULL, create_infer, NULL);
    pthread_create(&threads[3], NULL, picture2, NULL);
  //  pthread_create(&threads[4], NULL, infer_labels, NULL);
    // pthread_create(&threads[3], NULL, picture, NULL);

    while (1)
        ;

    delete (camera_ptr);

    return 0;
}
