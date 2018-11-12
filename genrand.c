#include <stdio.h>
#include <math.h>

int main(int argc, char *argv[])
{
    if(argc != 2)
    {
        fprintf(stderr, "Usage: %s EDGES\n", argv[0]);
        return 1;
    }

    int edges = atoi(argv[1]);

    srand(time(0));

    printf("a b\n");
    printf("a b 1\n");
    for(int i = 0; i < edges; i++)
    {
        char a, b;
        a = rand() % 26;
        do {
            b = rand() % 26;
        } while(b == a);

        printf("%c %c 1\n", 'a' + a, 'a' + b);
    }
}
