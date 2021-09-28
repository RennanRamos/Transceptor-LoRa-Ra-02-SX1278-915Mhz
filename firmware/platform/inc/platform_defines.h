#ifndef PLATFORM_DEFINES_H_
#define PLATFORM_DEFINES_H_

/*****************************************************************************/

typedef void(*callback_t)(void);

/*****************************************************************************/

class Callback
{
public:
  virtual void execute(void) = 0;
};

/*****************************************************************************/

template<typename T>
class GenericCallback : public Callback
{
public:
  GenericCallback(T* object_ = nullptr, \
                 void(T:: *method_)(void) = nullptr):
                 object(object_), method(method_){}
  void execute(void) {(object->*method)();}
private:
  T* object;
  void(T:: *method)(void);
};

/*****************************************************************************/

class PlainCallback : public Callback
{
public:
  PlainCallback(callback_t callback_){callback = callback_;}
  void execute(void){callback();}
private:
  callback_t callback;
};

/*****************************************************************************/

#endif  /* PLATFORM_DEFINES_H_ */
