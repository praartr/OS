#include<stdio.h>
#include<errno.h>
#include<sys/syscall.h>

#define deepsleep() syscall(328)

int main()
{
printf("goodnight, Irene\n");
deepsleep();
printf("oops...woke up!\n");
}
