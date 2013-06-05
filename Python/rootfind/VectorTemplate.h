#ifndef MATH_VECTOR_TEMPLATE_H
#define MATH_VECTOR_TEMPLATE_H

#include "math.h"
#include <assert.h>
#include <iostream>

namespace Math {

/** @ingroup Math
 * @brief An iterator through VectorTemplate elements.
 *
 * Operates as an STL random access, input/output iterator.
 */
template <class T>
class VectorIterator
{
public:
  typedef VectorIterator<T> MyT;
  typedef T value_type;
  typedef T* pointer;
  typedef T& reference;
  typedef size_t difference_type;
  typedef std::random_access_iterator_tag iterator_category;

  inline VectorIterator() :ptr(NULL),stride(0) {}
  inline VectorIterator(const MyT& i) :ptr(i.ptr),stride(i.stride) {}
  inline VectorIterator(T* _ptr,int _stride) :ptr(_ptr),stride(_stride) {}
  inline T& operator*() { return *ptr; }
  inline T* operator->() { return ptr; }
  inline MyT& operator ++() { ptr+=stride; return *this; }
  inline MyT& operator --() { ptr-=stride; return *this; }
  inline MyT& operator ++(int) { ptr+=stride; return *this; }
  inline MyT& operator --(int) { ptr-=stride; return *this; }
  //inline MyT& operator +=(int i) { ptr+=i*stride; return *this; }
  //inline MyT& operator -=(int i) { ptr-=i*stride; return *this; }
  inline MyT operator +(int i) const { return MyT(ptr+i*stride,stride); }
  inline MyT operator -(int i) const { return MyT(ptr-i*stride,stride); }
  inline bool operator !=(const MyT& i) { return ptr!=i.ptr; }
  inline bool operator ==(const MyT& i) { return ptr==i.ptr; }
  inline bool operator < (const MyT& i) { return ptr<i.ptr; }
  inline bool operator > (const MyT& i) { return ptr>i.ptr; }
  inline size_t operator - (const MyT& i) const { return (ptr-i.ptr)/stride; }

  T *ptr;
  int stride;
};

/** @ingroup Math
 * @brief A vector over the field T.
 *
 * This is a basic vector class used throughout the math library.
 * Element access is either through array notation v[i] or parentheses v(i).
 * The range of element indices is [0,v.n).
 *
 * The usual usage mode for a VectorTemplate is as both element storage and
 * manipulation.  It manages all the internal data, allowing resizing and
 * reallocation of the storage as necessary, and deleting internally managed
 * data on destruction.  By default, a VectorTemplate is "empty" and can be 
 * initialized by resizing it to the desired size, or by using it as the
 * result of an operator (e.g. a.add(b,c)).
 *
 * Once a VectorTemplate has been initialized, it should not be 
 * automatically resized by any function.  To resize, the resize() method
 * should be called directly.  To reset it to the empty state (and delete
 * all manged data) the clear() method should be used.
 *
 * A second usage mode is to point to data managed by some outside source.
 * Altering vector elements will alter the external data, and vice versa.
 * For example,
 *
 * @code
 * float a[4] = { 0,1,2,3 }
 * VectorTemplate<float> v;
 * v.setRef(a,4);       //v now points to the data in a
 * assert(v(0) == 0);   //v0 is a0
 * a[0] = 4;
 * assert(v(0) == 4);   //since a0 has changed, v0 changes as well
 * v[0] = 5;

 * assert(a[0] == 5);   //since v0 has changed, a0 changes as well
 * @endcode
 *
 * An empty VectorTemplate can be set to reference outside data using the
 * methods with "Ref" in their names.  In this mode, the size of the data
 * is fixed, and it cannot be reallocated, destroyed, or resized. 
 * To reset to the empty state, the clear() method should be used.
 * 
 *
 * Standard math operations are provided, add, subtract, multiply, etc.
 * These operations are most often implemented as result.op(arg1,arg2,...).
 * The reason for this interface rather than operators, like a = b+c,
 * is that it avoids unnecessary copying.  A smart compiler might be able
 * to optimize the code a = b+c to avoid copying the result of b+c into a,
 * but this is by no means necessarily implemented.  Specifying the
 * operation directly is necessarily efficient.
 *
 * When an operator is called on an empty vector, the vector is resized
 * to the proper dimensions.  If the vector is non-empty, the vector is
 * checked for the proper dimensions.  On incorrect dimensions, the
 * operator will abort.
 *
 *
 * Another feature is the ability to access elements stored 
 * non-contiguously in memory.  This is handy for accessing matrix 
 * columns or diagonals directly, as though they were normal vectors.
 * This depends on two additional parameters, 'base' and 'stride', that 
 * control how elements access the data in 'data'. Element i accesses
 * data[base+i*stride].  So to access every 2nd element of the data array,
 * set stride to 2, to access every 3rd, set stride to 3, etc.
 * It is important to make sure element accesses do not exceed the bounds
 * of the array, so the isValid() and isValidIndex() methods are handy
 * here.
 */
template <class T>
class VectorTemplate
{
public:
  typedef VectorTemplate<T> MyT;
  typedef VectorIterator<T> ItT;

  VectorTemplate();
  VectorTemplate(const MyT&);
  VectorTemplate(int n);
  VectorTemplate(int n, T initval);
  VectorTemplate(int n, const T* vals);
  ~VectorTemplate();

  inline T* getPointer() const { return vals; }
  inline int getCapacity() const { return capacity; }
  inline T* getStart() const { return vals+base; }
  inline int getSize() const { return n; }
  inline ItT begin() const { return ItT(vals+base,stride); }
  inline ItT end() const { return ItT(vals+base+n*stride,stride); }

  void resize(int size);
  void resize(int size, T initval);
  void clear();

  const MyT& operator = (const MyT& v);
  bool operator == (const MyT&) const;
  inline bool operator != (const MyT& a) const { return !operator==(a); }
  inline operator T* ();
  inline operator const T* () const;
  inline const T& operator() (int i) const { return operator[](i); }
  inline T& operator() (int i) { return operator[](i); }
  inline const T& operator[] (int i) const;
  inline T& operator[] (int i);
  inline void operator += (const MyT& a) { inc(a); }
  inline void operator -= (const MyT& a) { dec(a); }
  inline void operator *= (T c) { inplaceMul(c); }
  inline void operator /= (T c) { inplaceDiv(c); }

  void copy(const MyT&);
  template <class T2> void copy(const VectorTemplate<T2>&);
  void copy(const T* vals);
  void copySubVector(int i,const MyT&);
  void swap(MyT&);
  void swapCopy(MyT&);
  void add(const MyT&, const MyT&);
  void sub(const MyT&, const MyT&);
  void mul(const MyT&, T);
  void div(const MyT&, T);
  void axpby(T a,const MyT& x,T b,const MyT& y);
  //the following are increment-type methods
  void inc(const T&);
  void inc(const MyT&);
  void dec(const MyT&);
  void madd(const MyT&, T);

  void setRef(const MyT&,int base=0,int stride=1,int size=-1);
  void setRef(T* vals,int length,int base=0,int stride=1,int size=-1);
  void set(T);
  void setZero();
  void setNegative(const MyT&);
  void setNormalized(const MyT&);
  void setConjugate(const MyT&);

  void inplaceNegative();
  void inplaceMul(T);
  void inplaceDiv(T);
  void inplaceNormalize();
  void inplaceConjugate();

  void getCopy(MyT&) const;
  void getCopy(T* vals) const;
  void getSubVectorCopy(int i,MyT&) const;
  void getRef(MyT&,int base=0,int stride=1,int size=-1) const;
  inline void getNegative(MyT& v) const { v.setNegative(*this); }
  inline void getNormalized(MyT& v) const { v.setNormalized(*this); }
  inline void getConjugate(MyT& v) const { v.setConjugate(*this); }

  inline bool isReference() const { return !allocated; }
  inline bool isEmpty() const { return vals==NULL; }
  inline bool hasSize(int _size) const { return n==_size; }
  inline bool isValidIndex(int i) const { return i>=0 && i<n; }
  inline bool isCompact() const { return (stride==1); }
  bool isValid() const;
  bool isZero(T eps=0) const;
  bool isEqual(const MyT&,T eps=0) const;

  T dot(const MyT&) const;
  T dotSelf() const;
  T norm() const;
  T normSquared() const;
  T distance(const MyT&) const;
  T distanceSquared(const MyT&) const;
  T minElement(int* index=NULL) const;
  T maxElement(int* index=NULL) const;
  T minAbsElement(int* index=NULL) const;
  T maxAbsElement(int* index=NULL) const;

  void componentMul(const MyT& a,const MyT& b);
  void componentDiv(const MyT& a,const MyT& b);
  void componentMadd(const MyT& a,const MyT& b);
  void inplaceComponentMul(const MyT& c);
  void inplaceComponentDiv(const MyT& c);

  ///for each element xi, sets xi = f(ai)
  template <class UnaryOp>
  void componentOp(const MyT& a,UnaryOp& f);
  ///for each element xi, sets xi = f(ai,bi)
  template <class BinaryOp>
  void componentOp(const MyT& a,const MyT& b,BinaryOp& f);
  ///for each element xi, sets xi = f(ai,c)
  template <class BinaryOp>
  void componentOp(const MyT& a,T c,BinaryOp& f);

  ///for each element xi, sets xi = f(xi)
  template <class UnaryOp>
  void inplaceComponentOp(UnaryOp& f);
  ///for each element xi, sets xi = f(xi,ci)
  template <class BinaryOp>
  void inplaceComponentOp(const MyT& c,BinaryOp& f);
  ///for each element xi, sets xi = f(xi,c)
  template <class BinaryOp>
  void inplaceComponentOp(T c,BinaryOp& f);

private:
  //read only
  T* vals;
  int capacity;
  bool allocated;

public:
  //alterable
  int base,stride,n;
};

class Complex;
typedef class VectorTemplate<float> fVector;
typedef class VectorTemplate<double> dVector;
typedef class VectorTemplate<Complex> cVector;

template <class T>
std::ostream& operator << (std::ostream&, const VectorTemplate<T>&);
template <class T>
std::istream& operator >> (std::istream&, VectorTemplate<T>&);



template <class T>
template <class UnaryOp>
void VectorTemplate<T>::componentOp(const MyT& a,UnaryOp& f)
{
  if(isEmpty()) resize(a.n);
  else assert(hasSize(a.n));
  T* v=getStart();
  T* va=a.getStart();
  for(int i=0;i<n;i++,v+=stride,va+=a.stride)
    *v = f(*va);
}

template <class T>
template <class BinaryOp>
void VectorTemplate<T>::componentOp(const MyT& a,const MyT& b,BinaryOp& f)
{
  assert(a.hasSize(b.n));
  if(isEmpty()) resize(a.n);
  else assert(hasSize(a.n));
  T* v=getStart();
  T* va=a.getStart();
  T* vb=b.getStart();
  for(int i=0;i<n;i++,v+=stride,va+=a.stride,vb+=b.stride)
    *v = f(*va,*vb);
}

template <class T>
template <class BinaryOp>
void VectorTemplate<T>::componentOp(const MyT& a,T c,BinaryOp& f)
{
  if(isEmpty()) resize(a.n);
  else assert(hasSize(a.n));
  T* v=getStart();
  T* va=a.getStart();
  for(int i=0;i<n;i++,v+=stride,va+=a.stride)
    *v = f(*va,c);
}

template <class T>
template <class UnaryOp>
void VectorTemplate<T>::inplaceComponentOp(UnaryOp& f)
{
  assert(!isEmpty());
  T* v=getStart();
  for(int i=0;i<n;i++,v+=stride)
    *v = f(*v);
}

template <class T>
template <class BinaryOp>
void VectorTemplate<T>::inplaceComponentOp(const MyT& c,BinaryOp& f)
{
  assert(!isEmpty());
  assert(hasSize(c.n));
  T* v=getStart();
  T* vc=c.getStart();
  for(int i=0;i<n;i++,v+=stride,vc+=c.stride)
    *v = f(*v,*vc);
}

template <class T>
template <class BinaryOp>
void VectorTemplate<T>::inplaceComponentOp(T c,BinaryOp& f)
{
  assert(!isEmpty());
  T* v=getStart();
  for(int i=0;i<n;i++,v+=stride)
    *v = f(*v,c);
}

//VectorTemplate inlined methods
template <class T>
inline const T& VectorTemplate<T>::operator[](int i) const
{
	return vals[i*stride+base];
}

template <class T>
inline T& VectorTemplate<T>::operator[](int i)
{
	return vals[i*stride+base];
}

template <class T>
inline VectorTemplate<T>::operator T* ()
{
  assert(isCompact());
  return vals+base; 
}

template <class T>
inline VectorTemplate<T>::operator const T* () const
{
  assert(isCompact());
  return vals+base; 
}

template <class T>
inline bool FuzzyEquals(const VectorTemplate<T>& a, const VectorTemplate<T>& b,T eps)
{
  return a.isEqual(b,eps);
}

template <class T>
inline bool FuzzyZero(const VectorTemplate<T>& a,T eps)
{
  return a.isZero(eps);
}

template <class T>
inline T dot(const VectorTemplate<T>& a, const VectorTemplate<T>& b)
{
	return a.dot(b);
}

template <class T>
inline T norm(const VectorTemplate<T>& a)
{
	return a.norm();
}



///The following are potentially very inefficient, depending on the ability
///of your compiler to optimize the code to eliminate copy constructor calls.
template <class T>
inline VectorTemplate<T> operator + (const VectorTemplate<T>& a, const VectorTemplate<T>& b)
{
	VectorTemplate<T> v;
	v.add(a,b);
	return v;
}

template <class T>
inline VectorTemplate<T> operator - (const VectorTemplate<T>& a, const VectorTemplate<T>& b)
{
	VectorTemplate<T> v;
	v.sub(a,b);
	return v;
}

template <class T>
inline VectorTemplate<T> operator * (const VectorTemplate<T>& a, T c)
{
	VectorTemplate<T> v;
	v.mul(a,c);
	return v;
}

template <class T>
inline VectorTemplate<T> operator * (T c, const VectorTemplate<T>& a)
{
	VectorTemplate<T> v;
	v.mul(a,c);
	return v;
}

template <class T>
inline VectorTemplate<T> operator / (const VectorTemplate<T>& a, T c)
{
	VectorTemplate<T> v;
	v.div(a,c);
	return v;
}

template <class T>
inline VectorTemplate<T> operator / (T c, const VectorTemplate<T>& a)
{
	VectorTemplate<T> v;
	v.div(a,c);
	return v;
}

} //namespace Math

namespace std
{
  template<class T> inline void swap(Math::VectorTemplate<T>& a, Math::VectorTemplate<T>& b)
  {
    a.swap(b);
  }
} //namespace std


#endif
