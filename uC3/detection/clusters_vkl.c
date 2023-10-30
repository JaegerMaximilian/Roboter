/*
 * clusters_vkl.c
 *
 * Created: 09.05.2020 16:10:40
 *  Author: Richard
 */ 

#include "clusters_vkl.h"
#include "cluster.h"
#include <stdlib.h>
#include "usart.h"

// struct node_t* MakeNode(struct cluster* data) {
// 	struct node_t* newPtr = (struct node_t*)malloc(sizeof(struct node_t));
// 	if(newPtr == 0) {
// 		return 0;
// 	}
// 	newPtr->data = *data;
// 	newPtr->next = NULL;
// 	return newPtr;
// }
// void Append(struct node_t** ptr, struct node_t* newP) {
// 	if(*ptr == NULL) {
// 		*ptr = newP;
// 	}
// 	else {
// 		struct node_t* temp= *ptr;
// 		while(temp->next!=NULL){
// 			temp=temp->next;
// 		}
// 		
// 		temp->next = newP;
// 	}
// }

// int Makeandappendnode(struct node_t** ptr, struct cluster* data){
// 	struct node_t* newnode = MakeNode(data);
// 	if(newnode==0){
// 		return 0;
// 	}
// 	Append(ptr, newnode);
// 	return 1;
// }

// void deleteNode(struct node_t** listroot, struct cluster value)
// {
// 	
// 	struct node_t* lastnode = NULL, *currentnode = *listroot;
// 	
// 	char text[20];
// 	
// 	while(currentnode && currentnode->data.locx != value.locx && currentnode->data.locy != value.locy)
// 	{
// 		
// 		lastnode = currentnode;
// 		currentnode = currentnode->next;
// 	}
// 	
// 	if(currentnode == NULL)
// 	{
// 		
// 		//sprintf(text,"Thats the wrong Node\r\n");
// 	}
// 	else
// 	{
// 		
// 		if (lastnode == NULL)
// 		{
// 			
// 			*listroot = (*listroot)->next;
// 			lastnode->next = currentnode->next;
// 			//sprintf(text,"Ara\r\n");
// 		}
// 		else
// 		{
// 			
// 			lastnode->next = currentnode->next;
// 			//sprintf(text,"Ehre genommen\r\n");
// 			
// 		}
// 		
// 		//writeString_usart(&usartD0, text);
// 		free(currentnode);
// 		//writeString_usart(&usartD0, text);
// 	}
// }