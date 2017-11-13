#include<stdio.h>
#include<fcntl.h>
#include <linux/types.h> /* size_t */
int main()
{
	int fd1=0,fd2 = 0,ret=0;
	char buff[200]={0};
	
	fd1=open("/dev/my_axi1",O_RDWR);
	fd2=open("/dev/my_axi1",O_RDWR);
	
	printf("fd1 :%d\n",fd1);
	
	char charin[200]={0};
	
	scanf("%s",charin);

	int off = 0;
	ret=write(fd1,charin,strlen(charin));
	//printf("charin: %s ;length: %d bytes write_num:%d\n",charin,strlen(charin),ret);
	//int i=0;
        //ret=read(fd2,buff,50);				
	//printf("buff: %s ;length: %d bytes\n",buff,ret);
	
	

	close(fd1);
	close(fd2);
	
}

