#include<linux/linkage.h>
#include<linux/kernel.h>
#include<linux/syscalls.h>
#include<linux/wait.h>
#include<linux/sched.h>

extern unsigned long sum_of_services;
extern unsigned long sum_of_waits;
extern int number_of_requests_seen;
DECLARE_WAIT_QUEUE_HEAD(gone);
__sched sleep_on(wait_queue_head_t *q)
{
unsigned long flags;
wait_queue_t wait;

init_waitqueue_entry(&wait,current);
__set_current_state(TASK_UNINTERRUPTIBLE);
spin_lock_irqsave(&q->lock,flags);
__add_wait_queue(q,&wait);
spin_unlock(&q->lock);
schedule();
spin_lock_irq(&q->lock);
__remove_wait_queue(q,&wait);
spin_unlock_irqrestore(&q->lock,flags);
return;
}

SYSCALL_DEFINE1(goober,int,arg)
{
printk(KERN_ALERT"Hello from %d\n",arg);
return(1);
}
SYSCALL_DEFINE1(init_sigcounter,pid_t,pid)
{
int i;
unsigned long flags;
struct task_struct *p;
p=pid_task(find_vpid(pid),PIDTYPE_PID);
lock_task_sighand(p,&flags);
for(i=0;i<64;i++)
p->sighand->sigcounter[i]=0;
unlock_task_sighand(p,&flags);
return (0);
}
SYSCALL_DEFINE1(get_sigcounter,int,signumber)
{
int i,counter_value;
unsigned long flags;
struct task_struct *p;
p=pid_task(find_vpid(current->pid),PIDTYPE_PID);
lock_task_sighand(p,&flags);
counter_value = p->sighand->sigcounter[signumber];
unlock_task_sighand(p,&flags);
return counter_value;
}

SYSCALL_DEFINE0(deepsleep)
{
sleep_on(&gone);
}
SYSCALL_DEFINE0(init_disk)
{
sum_of_services = 0;
sum_of_waits = 0;
number_of_requests_seen = 0;
return 0;
}
SYSCALL_DEFINE0(disk_performance)
{
unsigned long mean_service_time = sum_of_services / number_of_requests_seen ;
unsigned long mean_waiting_time = sum_of_waits / number_of_requests_seen;
printk(KERN_ALERT "Mean service time is %lu\n",mean_service_time);
printk(KERN_ALERT "Mean waiting time is %lu\n",mean_waiting_time);
}
