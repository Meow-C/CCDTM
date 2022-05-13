#include <iostream>
#include <thread>
#include <unistd.h>
#include <sys/time.h>
// #include <time.h>/<ctime>    nanosleep()

void thread_task(int i)
{
    std::cout << i << ": begin.\n";
//    std::this_thread::sleep_for(std::chrono::seconds(10));
    sleep(1);  // usleep(useconds)
    std::cout << i << ": end.\n";
}

int main ()
{
    struct timeval tStartTime, tEndTime;
    clock_t  start,end;

    std::thread thread;
    start = clock();
    gettimeofday(&tStartTime,NULL);
    for (int i = 0; i < 4; i++) {
        thread = std::thread(thread_task, i);
        thread.join();
    }

    gettimeofday(&tEndTime,NULL);
    end = clock();
    float dCostTime = tEndTime.tv_sec - tStartTime.tv_sec + float(tEndTime.tv_usec - tStartTime.tv_usec)/1000000;
    std::cout << "running time: " << double(end-start)/CLOCKS_PER_SEC << " s \n";
    std::cout << "running time(using gettimeofday()): " << dCostTime << " s \n";

    std::thread threads[4];
    std::cout << "Spawning 4 threads...\n";
    start = clock();
    for (int i = 0; i < 4; i++) {
        threads[i] = std::thread(thread_task,i);
    }
    for (auto& t : threads) {
        t.join();
    }
    end = clock();
    std::cout << "All threads joined.\n";
    std::cout << "running time: " << double(end-start)/CLOCKS_PER_SEC << " s \n";

    return EXIT_SUCCESS;
}

