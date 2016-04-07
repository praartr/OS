#include<stdio.h>
#include<sys/signal.h>
#include<errno.h>

#define init_sigcounter(pid) syscall(326,pid)
#define get_sigcounter(signumber) syscall(327,signumber)

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
			ret = get_sigcounter(SIGUSR1);
			printf("counter value for SIGUSR1 %d\n",ret);
			ret = get_sigcounter(SIGSTOP);
			printf("counter value for SIGSTOP %d\n",ret);
			ret= get_sigcounter(SIGCONT);
			printf("counter value for SIGCONT %d\n",ret);
			ret = get_sigcounter(SIGWINCH);
			printf("counter value for SIGWINCH %d\n",ret);
			ret = get_sigcounter(SIGURG);
			printf("counter value for SIGURG %d\n",ret);
			sleep(5);
	}

	break;
	default:

		ret =init_sigcounter(pid);
		while(1){
			sleep(10);
			ret = kill(pid,SIGUSR1);
			
			ret = kill(pid,SIGSTOP);
			ret = kill(pid,SIGCONT);
			ret = kill(pid,SIGWINCH);
			ret = kill(pid,SIGURG);
			}
		printf("success%d",ret);
}
return 0;
}
