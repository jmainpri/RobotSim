#include "metric.h"
#include "complex.h"
#include <assert.h>

namespace Math {

template <class T>
inline T DotSelf(const T& a) { return dot(a,a); }

#define V VectorTemplate
#define M MatrixTemplate

inline double Pow(double a,float b) { return Pow(a,(double)b); }
inline float Pow(float a,double b) { return Pow(a,(float)b); }

template<class T>
T Norm(const V<T>& x,Real norm)
{
  if(norm==One) return Norm_L1(x);
  else if(norm==Two) return Norm_L2(x);
  else if(IsInf(norm)) return Norm_LInf(x);
  else {
    assert(norm > 0);
    T sum=0;
    for(int i=0;i<x.n;i++)
      sum += Pow(x(i),norm);
    return Pow(sum,Inv(norm));
  }
}

template <class T>
T Norm_L1(const V<T>& x)
{
  T sum=0;
  for(int i=0;i<x.n;i++)
    sum += Abs(x[i]);
  return sum;
}

template <class T>
T Norm_L2(const V<T>& x)
{
  T sum=0;
  for(int i=0;i<x.n;i++)
    sum += DotSelf(x[i]);
  return Sqrt(sum);
}

template <class T>
T Norm_L2_Safe(const V<T>& x)
{
  //factor out maximum element to avoid overflow
  T xmax = x.maxAbsElement();
  if(xmax == 0) return 0;
  T sum=0;
  for(int i=0;i<x.n;i++)
    sum += DotSelf(x[i]/xmax);
  return xmax*Sqrt(sum);
}

template <class T>
T Norm_LInf(const V<T>& x)
{
  T sum=0;
  for(int i=0;i<x.n;i++)
    sum = Max(Abs(x[i]),sum);
  return sum;
}

template <class T>
T Norm_Mahalanobis(const V<T>& x,const M<T>& A)
{
  V<T> temp;
  A.mul(x,temp);
  return Sqrt(x.dot(temp));
}

template <class T>
T Norm_Weighted(const V<T>& x,Real norm,const V<T>& w)
{
  if(norm==One) return Norm_WeightedL1(x,w);
  else if(norm==Two) return Norm_WeightedL2(x,w);
  else if(IsInf(norm)) return Norm_WeightedLInf(x,w);
  else {
    assert(w.n==x.n);
    T sum=0;
    for(int i=0;i<x.n;i++) 
      sum += w(i)*Pow(x(i),norm);
    return Pow(sum,Inv(norm));
  }
}

template <class T>
T Norm_WeightedL1(const V<T>& x,const V<T>&w)
{
  assert(w.n==x.n);
  T sum=0;
  for(int i=0;i<x.n;i++)
    sum += w(i)*Abs(x[i]);
  return sum;
}

template <class T>
T Norm_WeightedL2(const V<T>& x,const V<T>&w)
{
  assert(w.n==x.n);
  T sum=0;
  for(int i=0;i<x.n;i++)
    sum += w(i)*DotSelf(x[i]);
  return Sqrt(sum);
}

template <class T>
T Norm_WeightedLInf(const V<T>& x,const V<T>&w)
{
  assert(w.n==x.n);
  T sum=0;
  for(int i=0;i<x.n;i++)
    sum = Max(w(i)*Abs(x[i]),sum);
  return sum;
}

template<class T>
T Distance(const V<T>& x,const V<T>& y,Real norm)
{
  if(norm==One) return Distance_L1(x,y);
  else if(norm==Two) return Distance_L2(x,y);
  else if(IsInf(norm)) return Distance_LInf(x,y);
  else {
    assert(norm > 0);
    assert(x.n == y.n);
    T sum=0;
    for(int i=0;i<x.n;i++)
      sum += Pow(x(i)-y(i),norm);
    return Pow(sum,Inv(norm));
  }
}

template<class T>
T Distance_L1(const V<T>& x,const V<T>& y)
{
  assert(x.n == y.n);
  T sum=0;
  for(int i=0;i<x.n;i++)
    sum += Abs(x[i]-y[i]);
  return sum;
}

template<class T>
T Distance_L2(const V<T>& x,const V<T>& y)
{
  assert(x.n == y.n);
  T sum=0;
  for(int i=0;i<x.n;i++)
    sum += DotSelf(x[i]-y[i]);
  return Sqrt(sum);
}

template<class T>
T Distance_L2_Safe(const V<T>& x,const V<T>& y)
{
  assert(x.n == y.n);
  T dmax = 0;
  for(int i=0;i<x.n;i++) dmax = Max(dmax,Abs(x[i]-y[i]));
  if(dmax == 0) return 0;
  T sum=0;
  for(int i=0;i<x.n;i++)
    sum += DotSelf((x[i]-y[i])/dmax);
  return dmax*Sqrt(sum);
}

template<class T>
T Distance_LInf(const V<T>& x,const V<T>& y)
{
  assert(x.n == y.n);
  T sum=0;
  for(int i=0;i<x.n;i++)
    sum = Max(Abs(x[i]-y[i]),sum);
  return sum;
}

template<class T>
T Distance_Mahalanobis(const V<T>& x,const V<T>& y,const M<T>& A)
{
  assert(x.n == y.n);
  V<T> d;
  d.sub(x,y);
  return Norm_Mahalanobis(d,A);
}

template <class T>
T Distance_Weighted(const V<T>& x,const V<T>& y,Real norm,const V<T>& w)
{
  if(norm==One) return Distance_WeightedL1(x,y,w);
  else if(norm==Two) return Distance_WeightedL2(x,y,w);
  else if(IsInf(norm)) return Distance_WeightedLInf(x,y,w);
  else {
    assert(w.n==x.n);
    T sum=0;
    for(int i=0;i<x.n;i++) 
      sum += w(i)*Pow(x(i)-y(i),(T)norm);
    return Pow(sum,Inv(norm));
  }
}

template <class T>
T Distance_WeightedL1(const V<T>& x,const V<T>& y,const V<T>&w)
{
  assert(x.n == y.n);
  assert(w.n==x.n);
  T sum=0;
  for(int i=0;i<x.n;i++)
    sum += w(i)*Abs(x[i]-y[i]);
  return sum;
}

template <class T>
T Distance_WeightedL2(const V<T>& x,const V<T>& y,const V<T>&w)
{
  assert(x.n == y.n);
  assert(w.n==x.n);
  T sum=0;
  for(int i=0;i<x.n;i++)
    sum += w(i)*Sqr(x[i]-y[i]);
  return Sqrt(sum);
}

template <class T>
T Distance_WeightedLInf(const V<T>& x,const V<T>& y,const V<T>&w)
{
  assert(x.n == y.n);
  assert(w.n==x.n);
  T sum=0;
  for(int i=0;i<x.n;i++)
    sum = Max(w(i)*Abs(x[i]-y[i]),sum);
  return sum;
}


template <class T>
T Norm_L1(const M<T>& A)
{
  T v=0;
  for(int j=0;j<A.n;j++) {
    T sum=0;
    for(int i=0;i<A.m;i++) sum += Abs(A(i,j));
    v = Max(v,sum);
  }
  return v;
}

template <class T>
T Norm_LInf(const M<T>& A)
{
  T v=0;
  for(int i=0;i<A.m;i++) {
    T sum=0;
    for(int j=0;j<A.m;j++) sum += Abs(A(i,j));
    v = Max(v,sum);
  }
  return v;
}

template <class T>
T Norm_Frobenius(const M<T>& A)
{
  T sum=0;
  MatrixIterator<T> a = A.begin();
  for(int i=0;i<A.m;i++,a.nextRow())
    for(int j=0;j<A.n;j++,a.nextCol())
      sum += DotSelf(*a);
  return Sqrt(sum);
}

template <class T>
T Norm_Frobenius_Safe(const M<T>& A)
{
  MatrixIterator<T> a = A.begin();
  T vmax=A.maxAbsElement();
  if(vmax == 0) return 0;
  T sum=0;
  a=A.begin();
  for(int i=0;i<A.m;i++,a.nextRow())
    for(int j=0;j<A.n;j++,a.nextCol())
      sum += DotSelf(*a/vmax);
  return vmax*Sqrt(sum);
}


template<class T>
T Distance_L1(const M<T>& A,const M<T>& B)
{
  assert(A.hasDims(B.m,B.n));
  T v=0;
  for(int j=0;j<A.n;j++) {
    T sum=0;
    for(int i=0;i<A.m;i++) sum += Abs(A(i,j)-B(i,j));
    v = Max(v,sum);
  }
  return v;
}


template<class T>
T Distance_LInf(const M<T>& A,const M<T>& B)
{
  T v=0;
  for(int i=0;i<A.m;i++) {
    T sum=0;
    for(int j=0;j<A.m;j++) sum += Abs(A(i,j)-B(i,j));
    v = Max(v,sum);
  }
  return v;
}

template<class T>
T Distance_Frobenius(const M<T>& A,const M<T>& B)
{
  assert(A.hasDims(B.m,B.n));
  MatrixIterator<T> a = A.begin();
  MatrixIterator<T> b = B.begin();
  T sum=0;
  for(int i=0;i<A.m;i++,a.nextRow(),b.nextRow())
    for(int j=0;j<A.n;j++,a.nextCol(),b.nextCol())
      sum += DotSelf(*a-*b);
  return Sqrt(sum);
}

template<class T>
T Distance_Frobenius_Safe(const M<T>& A,const M<T>& B)
{
  MatrixIterator<T> a = A.begin();
  MatrixIterator<T> b = B.begin();
  T vmax=0;
  for(int i=0;i<A.m;i++,a.nextRow(),b.nextRow())
    for(int j=0;j<A.n;j++,a.nextCol(),b.nextCol())
      vmax = Max(vmax,Abs(*a-*b));
  if(vmax == 0) return 0;
  T sum=0;
  for(int i=0;i<A.m;i++,a.nextRow(),b.nextRow())
    for(int j=0;j<A.n;j++,a.nextCol(),b.nextCol())
      sum += Sqr((*a-*b)/vmax);
  return vmax*Sqrt(sum);
}



//instantiations for complex
template <> Complex Norm_LInf(const V<Complex>& x)
{
  Real sum=Zero;
  for(int i=0;i<x.n;i++)
    sum = Max(Abs(x[i]),sum);
  return sum;
}

template <> Complex Norm_WeightedLInf(const V<Complex>& x,const V<Complex>&w)
{
  assert(w.n==x.n);
  Real sum=0;
  for(int i=0;i<x.n;i++)
    sum = Max(Abs(w(i))*Abs(x[i]),sum);
  return sum;
}

template <> Complex Distance_LInf(const V<Complex>& x,const V<Complex>& y)
{
  assert(x.n == y.n);
  Real sum=Zero;
  for(int i=0;i<x.n;i++)
    sum = Max(Abs(x[i]-y[i]),sum);
  return sum;
}

template <> Complex Distance_WeightedLInf(const V<Complex>& x,const V<Complex>& y,const V<Complex>& w)
{
  assert(x.n == y.n);
  Real sum=Zero;
  for(int i=0;i<x.n;i++)
    sum = Max(Abs(w[i])*Abs(x[i]-y[i]),sum);
  return sum;
}


template <> Complex Distance_L2_Safe(const V<Complex>& x,const V<Complex>& y)
{
  assert(x.n == y.n);
  Real dmax = Zero;
  for(int i=0;i<x.n;i++) dmax = Max(dmax,Abs(x[i]-y[i]));
  if(dmax == Zero) return Zero;
  Complex sum=Zero;
  for(int i=0;i<x.n;i++)
    sum += DotSelf((x[i]-y[i])/dmax);
  return dmax*Sqrt(sum);
}

template <> Complex Norm_L1(const M<Complex>& A)
{
  Real v=Zero;
  for(int j=0;j<A.n;j++) {
    Real sum=Zero;
    for(int i=0;i<A.m;i++) sum += Abs(A(i,j));
    v = Max(v,sum);
  }
  return v;
}

template <> Complex Norm_LInf(const M<Complex>& A)
{
  Real v=Zero;
  for(int i=0;i<A.m;i++) {
    Real sum=Zero;
    for(int j=0;j<A.m;j++) sum += Abs(A(i,j));
    v = Max(v,sum);
  }
  return v;
}

template <> Complex Distance_L1(const M<Complex>& A,const M<Complex>& B)
{
  assert(A.hasDims(B.m,B.n));
  Real v=Zero;
  for(int j=0;j<A.n;j++) {
    Real sum=Zero;
    for(int i=0;i<A.m;i++) sum += Abs(A(i,j)-B(i,j));
    v = Max(v,sum);
  }
  return v;
}


template <> Complex Distance_LInf(const M<Complex>& A,const M<Complex>& B)
{
  Real v=Zero;
  for(int i=0;i<A.m;i++) {
    Real sum=Zero;
    for(int j=0;j<A.m;j++) sum += Abs(A(i,j)-B(i,j));
    v = Max(v,sum);
  }
  return v;
}

template<> Complex Distance_Frobenius_Safe(const M<Complex>& A,const M<Complex>& B)
{
  MatrixIterator<Complex> a = A.begin();
  MatrixIterator<Complex> b = B.begin();
  Real vmax=0;
  for(int i=0;i<A.m;i++,a.nextRow(),b.nextRow())
    for(int j=0;j<A.n;j++,a.nextCol(),b.nextCol())
      vmax = Max(vmax,Abs(*a-*b));
  if(vmax == Zero) return Zero;
  Complex sum=Zero;
  for(int i=0;i<A.m;i++,a.nextRow(),b.nextRow())
    for(int j=0;j<A.n;j++,a.nextCol(),b.nextCol())
      sum += Sqr((*a-*b)/vmax);
  return vmax*Sqrt(sum);
}


#define DEFINEMETRIC(T) \
  template T Norm(const V<T>& x,Real norm);	  \
  template T Norm_L1(const V<T>& x); \
  template T Norm_L2(const V<T>& x); \
  template T Norm_L2_Safe(const V<T>& x); \
  template T Norm_LInf(const V<T>& x); \
  template T Norm_Mahalanobis(const V<T>& x,const M<T>& k); \
  template T Norm_Weighted(const V<T>& x,Real norm,const V<T>& w);	  \
  template T Norm_WeightedL1(const V<T>& x,const V<T>& w);		\
  template T Norm_WeightedL2(const V<T>& x,const V<T>& w); \
  template T Norm_WeightedLInf(const V<T>& x,const V<T>& w); \
  template T Distance(const V<T>& x,const V<T>& y,Real norm); \
  template T Distance_L1(const V<T>& x,const V<T>& y); \
  template T Distance_L2(const V<T>& x,const V<T>& y); \
  template T Distance_L2_Safe(const V<T>& x,const V<T>& y); \
  template T Distance_LInf(const V<T>& x,const V<T>& y); \
  template T Distance_Mahalanobis(const V<T>& x,const V<T>& y,const M<T>& A); \
  template T Distance_Weighted(const V<T>& x,const V<T>& y,Real norm,const V<T>& w); \
  template T Distance_WeightedL1(const V<T>& x,const V<T>& y,const V<T>& w); \
  template T Distance_WeightedL2(const V<T>& x,const V<T>& y,const V<T>& w); \
  template T Distance_WeightedLInf(const V<T>& x,const V<T>& y,const V<T>& w); \
  template T Norm_L1(const M<T>& A); \
  template T Norm_LInf(const M<T>& A); \
  template T Norm_Frobenius(const M<T>& A); \
  template T Norm_Frobenius_Safe(const M<T>& A); \
  template T Distance_L1(const M<T>& A,const M<T>& B); \
  template T Distance_LInf(const M<T>& A,const M<T>& B); \
  template T Distance_Frobenius(const M<T>& A,const M<T>& B); \
  template T Distance_Frobenius_Safe(const M<T>& A,const M<T>& B);

DEFINEMETRIC(float);
DEFINEMETRIC(double);
//Curses, visual studio...
//DEFINEMETRIC(Complex);
  template Complex Norm(const V<Complex>& x,Real norm);	  
  template Complex Norm_L1(const V<Complex>& x); 
  template Complex Norm_L2(const V<Complex>& x); 
  template Complex Norm_L2_Safe(const V<Complex>& x); 
  template Complex Norm_Mahalanobis(const V<Complex>& x,const M<Complex>& k); 
  template Complex Norm_Weighted(const V<Complex>& x,Real norm,const V<Complex>& w);	  
  template Complex Norm_WeightedL1(const V<Complex>& x,const V<Complex>& w);		
  template Complex Norm_WeightedL2(const V<Complex>& x,const V<Complex>& w); 
  template Complex Distance(const V<Complex>& x,const V<Complex>& y,Real norm); 
  template Complex Distance_L1(const V<Complex>& x,const V<Complex>& y); 
  template Complex Distance_L2(const V<Complex>& x,const V<Complex>& y); 
  template Complex Distance_Mahalanobis(const V<Complex>& x,const V<Complex>& y,const M<Complex>& A); 
  template Complex Distance_Weighted(const V<Complex>& x,const V<Complex>& y,Real norm,const V<Complex>& w); 
  template Complex Distance_WeightedL1(const V<Complex>& x,const V<Complex>& y,const V<Complex>& w); 
  template Complex Distance_WeightedL2(const V<Complex>& x,const V<Complex>& y,const V<Complex>& w); 
  template Complex Norm_Frobenius(const M<Complex>& A); 
  template Complex Norm_Frobenius_Safe(const M<Complex>& A); 
  template Complex Distance_Frobenius(const M<Complex>& A,const M<Complex>& B);


} //namespace Math
