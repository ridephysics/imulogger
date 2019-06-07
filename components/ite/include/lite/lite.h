#ifndef LITE_LITE_H
#define LITE_LITE_H

#include <string.h>

/* Validate string, non NULL and not zero length */
static inline int string_valid(const char *s)
{
   return s && strlen(s);
}

/* Relaxed comparison, e.g., sys_string_match("small", "smaller") => TRUE */
static inline int string_match(const char *a, const char *b)
{
   size_t min = MIN(strlen(a), strlen(b));

   return !strncasecmp(a, b, min);
}

/* Strict comparison, e.g., sys_string_match("small", "smaller") => FALSE */
static inline int string_compare(const char *a, const char *b)
{
   return strlen(a) == strlen(b) && !strcmp(a, b);
}

/* Strict comparison, like sys_string_compare(), but case insensitive,
 * e.g., sys_string_match("small", "SmAlL") => TRUE
 */
static inline int string_case_compare(const char *a, const char *b)
{
   return strlen (a) == strlen (b) && !strcasecmp (a, b);
}

#endif /* LITE_LITE_H */
