#ifdef WIN32
#   ifdef TaskSpace_EXPORTS
#       define TaskSpace_API __declspec(dllexport)
#   else
#       define TaskSpace_API  __declspec(dllimport)
#   endif
#else
#   define TaskSpace_API
#endif // WIN32