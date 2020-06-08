#include "driver.h"

int32_t verbose = 0;
int32_t rseed   = 0;
int32_t tsize   = (10);
int32_t test = 0;

// clang-format off
#define Version "0.1"
static ArgOption args[] = {
  // Kind,        Method,		name,	    reqd,  variable,		help
  { KindOption,   Integer, 		"-v", 	    0,     &verbose, 		"Set verbosity level" },
  { KindOption,   Integer, 		"--seed", 	0,     &rseed,  		"Set random number seed" },
  { KindOption,   Integer, 		"-n",    	0,     &tsize,  		"Set random number seed" },
  { KindOption,   Integer, 		"--case",  	0,     &test,    		"Set random number seed" },
  { KindHelp,     Help, 	"-h" },
  { KindEnd }
};
// clang-format on

static ArgDefs argp = { args, "Main", Version, NULL };

int
main(int argc, char ** argv) {
    progname = argv[0];

    srand(rseed);
    srandom(rseed);

    FDBG_INIT_DEBUGGER;

    ArgParser * ap = createArgumentParser(&argp);
    if (parseArguments(ap, argc, argv)) {
        die("Error parsing arguments");
    }
    freeCommandLine();
    freeArgumentParser(ap);

    // code goes here
    tsize                 = 1 << tsize;
    const uint32_t nsizes = 1 << 20;
    uint32_t *     sizes  = (uint32_t *)mymmap_alloc(nsizes * sizeof(uint32_t));
    for (uint32_t i = 0; i < nsizes; i++) {
        sizes[i] = rand() % (63 * PAGE_SIZE);
        while (sizes[i] == 0) {
            sizes[i] = rand() % (63 * PAGE_SIZE);
        }
    }

    void ** free_arr = (void **)mymmap_alloc(tsize * sizeof(void *));
    for (int32_t i = 0; i < tsize; i++) {
        free_arr[i] = 0;
    }
    uint64_t start_cycles = 0, end_cycles = 0;
    int32_t  incr     = (256);
    uint32_t true_idx = 0;


    if (test == 0) {
        start_cycles = grabTSC();
        for (int32_t i = 0; i < tsize; i++) {
            free_arr[i] = alloc(sizes[(true_idx++) & (nsizes - 1)]);
            if (i == incr) {
                for (int32_t j = 0; j < incr; j++) {
                    dealloc(free_arr[j]);
                }
                incr += incr;
                i = (-1);
            }
        }
        for (int32_t i = 0; i < tsize; i++) {
            dealloc(free_arr[i]);
        }
        end_cycles = grabTSC();
    }
    else if (test == 1) {
        start_cycles = grabTSC();
        for (int32_t i = 0; i < tsize; i++) {
            free_arr[i] = malloc(sizes[(true_idx++) & (nsizes - 1)]);
            if (i == incr) {
                for (int32_t j = 0; j < incr; j++) {
                    free(free_arr[j]);
                }
                incr += incr;
                i = (-1);
            }
        }
        for (int32_t i = 0; i < tsize; i++) {
            free(free_arr[i]);
        }
        end_cycles = grabTSC();
    }
    else {
        void * ptrs[100000 / 2000];
        for (uint32_t i = 0; i < 100000; i += 2000) {
            ptrs[i / 2000] = alloc(i);
        }
        for (uint32_t i = 0; i < 100000; i += 2000) {
            dealloc(ptrs[i / 2000]);
        }
    }

    fprintf(stderr, "Cycles: %.8E\n", (double)(end_cycles - start_cycles));
    FDBG_FREE_DEBUGGER;
    return 0;
}
