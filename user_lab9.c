#include <stdio.h>
#include <sys/signal.h>

void happy()
{
printf("child is happy\n");
}
main()
{
int pid,ret;
switch(pid=fork()){
	case 0:
		signal(SIGUSR1,happy);
		while(1){
			printf("child is playing\n");
			sleep(1);
		}
	default:
		 while(1){
			printf("parent is going to sleep\n");
			sleep(10);
			printf("parent wakes up...check on child\n");
			ret = kill(pid,SIGUSR1);
			printf("kill returned %d\n");
		}
}
}
