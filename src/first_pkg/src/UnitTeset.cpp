#include <stdio.h>
#include "motorDriver.h"

int main ()
{
	printf("this is from unit testing\n");
	printInfo();
	if(returnSum(5,7)==12)
	{printf("Test1 PASS");}
	return 0;
}
