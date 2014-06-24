#ifndef HPPCOMMON_H
#define HPPCOMMON_H

#include <cstddef>

#define MEMBER_PTR(type, var)                   \
    protected:                                  \
    type var##_;                                \
public:                                         \
type& var() { return var##_; }                   \
type var() const { return var##_; }             \
void set_##var(type val) { var##_ = val; }      

#define MEMBER_VAR(type, var)                   \
    protected:                                  \
    type var##_;                                \
public:                                         \
type& var() { return var##_; }                  \
const type& var() const { return var##_; }      \
void set_##var(type val) { var##_ = val; }       


#define MEMBER_INIT_NULL(var) var##_(0)
#define MEMBER_INIT_ARG(var) var##_(_##var)
#define MEMBER_INIT(var, val) var##_(val)

#define MEMBER_RELEASE_PTR(var) do { if (var##_) { delete var##_; var##_ = NULL;}  } while(0)


#endif // #ifndef HPPCOMMON_H

