//include <stdlib.h>
#include <string.h>
//include <sys/types.h>
#include <stdio.h>
//include <unistd.h>
// shared memory and semaphores
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <errno.h>

int semid;
key_t semkey = 16789;
int shmid;
key_t key = 15432;
char *shm;
// define operations to be performed on semaphores
struct sembuf p = {0, -1, SEM_UNDO | IPC_NOWAIT};   // acquire
struct sembuf v = {0, 1, SEM_UNDO};                 // release

// Sets default values

int main()
{
    
    printf("%s\n", "TEST Shm Client Start");

    // get semaphore instance
    if ((semid = semget(semkey, 1, 0666)) < 0) {
        printf("%s\n", "Error getting semaphore");
        printf("Error: %s\n", strerror(errno));
        return 0;
    }
    // get shared mem instance
    shmid = shmget(key, 1024, 0666);
    if ( shmid < 0) {
        printf("%s\n", "Error getting shared memory");
        printf("Error: %s\n", strerror(errno));
        return 0;
    }
    // attach shared memory to this process's address space
    shm = (char*)shmat(shmid, NULL, 0);
    if (shm == (char*)-1) {
        printf("%s\n", "Error attaching shared memory");
        return  0;
    }
}

