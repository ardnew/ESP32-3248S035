#ifndef callback_h
#define callback_h

// These templates allow us to bind instance methods (whose signatures are
// elaborated with template parameters) to objects via std::bind, and use these
// bindings as ordinary C callback functions.
//
// Using this approach allows us to avoid using C functions that refer to
// methods of global library objects (e.g., Arduino often uses a convention such
// as "extern TwoWire Wire", where "Wire" is then called from C ISRs).

template <typename T>
struct Callback;

template <typename Ret, typename... Params>
struct Callback<Ret(Params...)> {
   template <typename... Args>
   static Ret callback(Args... args) { return fn(args...); }
   static std::function<Ret(Params...)> fn;
};

template <typename Ret, typename... Params>
std::function<Ret(Params...)> Callback<Ret(Params...)>::fn;

#endif // callback_h
