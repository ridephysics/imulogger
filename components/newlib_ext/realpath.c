/* realpath.c - Return the canonicalized absolute pathname */

/* Written 2000 by Werner Almesberger */


#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <limits.h>
#include <errno.h>
#include <sys/stat.h>


/* FIXME: buffer overrun possible, loops forever on cyclic symlinks */


/*
 * Canonical name: never ends with a slash
 */

static int resolve_path(char *path,char *result,char *pos)
{
    if (*path == '/') {
	*result = '/';
	pos = result+1;
	path++;
    }
    *pos = 0;
    if (!*path) return 0;
    while (1) {
	char *slash;

	slash = *path ? strchr(path,'/') : NULL;
	if (slash) *slash = 0;
	if (!path[0] || (path[0] == '.' &&
	  (!path[1] || (path[1] == '.' && !path[2])))) {
	    pos--;
	    if (pos != result && path[0] && path[1])
		while (*--pos != '/');
	}
	else {
	    strcpy(pos,path);
	    pos = strchr(result,0);
	}
	if (slash) {
	    *pos++ = '/';
	    path = slash+1;
	}
	*pos = 0;
	if (!slash) break;
    }
    return 0;
}


char *realpath(const char *__restrict path,char *__restrict resolved_path)
{
    char cwd[PATH_MAX];
    char *path_copy;
    int res;

    if (!*path) {
	errno = ENOENT; /* SUSv2 */
	return NULL;
    }
    // we don't support cwd, so just assume it's '/'
    strcpy(cwd,"/");
    strcpy(resolved_path,"/");
    if (resolve_path(cwd,resolved_path,resolved_path)) return NULL;
    strcat(resolved_path,"/");
    path_copy = strdup(path);
    if (!path_copy) return NULL;
    res = resolve_path(path_copy,resolved_path,strchr(resolved_path,0));
    free(path_copy);
    if (res) return NULL;
    return resolved_path;
}
