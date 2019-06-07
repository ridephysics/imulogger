#ifndef NEWLIB_EXT_H
#define NEWLIB_EXT_H

#include <dirent.h>

int scandir(const char *dirp, struct dirent ***namelist,
              int (*filter)(const struct dirent *),
              int (*compar)(const struct dirent **, const struct dirent **));

int alphasort(const struct dirent **a, const struct dirent **b);

#endif /* NEWLIB_EXT_H */
