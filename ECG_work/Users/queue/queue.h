//
// Created by Lyt on 2024/12/25.
//

#ifndef FINAL_TASK_QUEUE_H
#define FINAL_TASK_QUEUE_H


extern struct queue testQ;
extern void queueInit(struct queue*, int [], int);
extern void queuePush(struct queue* q, int element);
extern int queuePop(struct queue* q);
extern void queuePrint(struct queue* q);

struct queue {
    int count;
    int* array;
    int front;
    int rear;
    int maxSize;
};

#endif //FINAL_TASK_QUEUE_H
