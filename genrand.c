#include <stdio.h>
#include <math.h>

int main(int argc, char *argv[])
{
    if(argc != 3)
    {
        fprintf(stderr, "Usage: %s MAXNODES EDGES\n", argv[0]);
        return 1;
    }

    int maxnodes = atoi(argv[1]);
    int edges = atoi(argv[2]);

    srand(time(0));

    printf("1 2\n");
    printf("1 2 1\n");
    for(int i = 0; i < edges; i++)
    {
        int a, b;
        a = rand() % maxnodes;
        do {
            b = rand() % maxnodes;
        } while(b == a);

        printf("%d %d 1\n", a, b);
    }
}
