#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
 
typedef struct { uint8_t v; uint8_t c; } vcount;
 
int cmp_val(const void *a, const void *b)
{
	int8_t x = *(uint8_t*)a - *(uint8_t*)b;
	return x < 0 ? -1 : x > 0;
}
 
int vc_cmp(const void *a, const void *b)
{
	return ((vcount*)b)->c - ((vcount*)a)->c;
}
 
int8_t get_mode(uint8_t* x, uint8_t len, vcount **list)
{
	int8_t i, j;
	vcount *vc;
 
	/* sort values */
	qsort(x, len, sizeof(uint8_t), cmp_val);
 
	/* count occurence of each value */
	for (i = 0, j = 1; i < len - 1; i++, j += (x[i] != x[i + 1]));
 
	*list = vc = malloc(sizeof(vcount) * j);
	vc[0].v = x[0];
	vc[0].c = 1;
 
	/* generate list value-count pairs */
	for (i = j = 0; i < len - 1; i++, vc[j].c++)
		if (x[i] != x[i + 1]) vc[++j].v = x[i + 1];
 
	/* sort that by count in descending order */
	qsort(vc, j + 1, sizeof(vcount), vc_cmp);
 
	/* the number of entries with same count as the highest */
	for (i = 0; i <= j && vc[i].c == vc[0].c; i++);
 
	return i;
}
 
int8_t main()
{
	uint8_t values[] = { 1, 3, 6, 6, 6, 6, 7, 7, 12, 12, 12, 12, 17 };
#	define len sizeof(values)/sizeof(uint8_t)
	vcount *vc;
 
	int8_t i, n_modes = get_mode(values, len, &vc);
 
	printf("got %d modes:\n", n_modes);
	for (i = 0; i < n_modes; i++)
		printf("\tvalue = %d, count = %d\n", vc[i].v, vc[i].c);
 
	free(vc);
	return 0;
}
