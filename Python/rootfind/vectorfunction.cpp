#include "vectorfunction.h"
#include "metric.h"
#include <assert.h>
using namespace Math;
using namespace std;

std::string NormScalarFieldFunction::Label() const
{
  if(degree == Two) return "L2Norm";
  else if(degree == One) return "L1Norm";
  else if(degree == Inf) return "LInfNorm";
  else {
    char buf[32];
    sprintf(buf,"%f",degree);
    std::string str;
    str = "L("; str+=buf; str+=")Norm";
    return str;
  }
}

void NormScalarFieldFunction::PreEval(const Vector& x)
{
  norm=Norm(x,degree);
}

Real NormScalarFieldFunction::Eval(const Vector& x) 
{
  return norm;
}

void NormScalarFieldFunction::Gradient(const Vector& x,Vector& grad) 
{
  if(degree == Two) {
    if(norm == Zero) grad.setZero();
    else grad.div(x,norm);
  }
  else
    AssertNotReached();
}

Real NormScalarFieldFunction::Gradient_i(const Vector& x,int i) 
{
  if(degree == Two) {
    if(norm == Zero) return Zero;
    return x(i)/norm;
  }
  else
    AssertNotReached();
  return Zero;
}

void NormScalarFieldFunction::Hessian(const Vector& x,Matrix& H)
{
  for(int i=0;i<x.n;i++) {
    for(int j=0;j<x.n;j++) {
      if(i==j) 
	H(i,j) = (One - Sqr(x(i)/norm))/norm;
      else
	H(i,j) = -(x(i)/norm)*(x(j)/norm)/norm;
    }
  }
}

Real NormScalarFieldFunction::Hessian_ij(const Vector& x,int i,int j)
{
  if(i==j)
    return (One - Sqr(x(i)/norm))/norm;
  else
    return -(x(i)/norm)*(x(j)/norm)/norm;
}

void MinimumScalarFieldFunction::PreEval(const Vector& x) { xmin = x.minElement(&index); }
Real MinimumScalarFieldFunction::Eval(const Vector& x) { return xmin; }
void MinimumScalarFieldFunction::Gradient(const Vector& x,Vector& grad) { grad.setZero(); grad(index)=One; }
Real MinimumScalarFieldFunction::Gradient_i(const Vector& x,int i) { return Delta(i,index); }

void MaximumScalarFieldFunction::PreEval(const Vector& x) { xmax = x.maxElement(&index); }
Real MaximumScalarFieldFunction::Eval(const Vector& x) { return xmax; }
void MaximumScalarFieldFunction::Gradient(const Vector& x,Vector& grad) { grad.setZero(); grad(index)=One; }
Real MaximumScalarFieldFunction::Gradient_i(const Vector& x,int i) { return Delta(i,index); }


std::string ComposeScalarFieldFunction::Label() const
{
  std::string sf=f->Label(), sg=g->Label();
  std::string str=sf;
  str+="(";
  str+=sg;
  str+="(x))";
  return str;
}

void ComposeScalarFieldFunction::PreEval(const Vector& x) 
{
  g->PreEval(x);
  gx=g->Eval(x);
  f->PreEval(gx);
}

void ComposeScalarFieldFunction::Gradient(const Vector& x,Vector& grad) 
{ 
  g->Gradient(x,grad);
  grad *= f->Deriv(gx);
}

Real ComposeScalarFieldFunction::Gradient_i(const Vector& x,int i) 
{ 
  return f->Deriv(gx)*g->Gradient_i(x,i);
}

void ComposeScalarFieldFunction::Hessian(const Vector& x,Matrix& H)
{
  //f''*g'*g'^t + f'*g''
  assert(H.m == x.n);
  assert(H.n == x.n);

  Vector grad(x.n);
  Real df = f->Deriv(gx);
  Real ddf = f->Deriv2(gx);
  g->Gradient(x,grad);
  g->Hessian(x,H);
  H.inplaceMul(df);

  for(int i=0;i<H.m;i++)
    for(int j=0;j<H.n;j++)
      H(i,j) += ddf*grad(i)*grad(j);
}

Real ComposeScalarFieldFunction::Hessian_ij(const Vector& x,int i,int j)
{
  return f->Deriv(gx)*g->Hessian_ij(x,i,j) + f->Deriv2(gx)*g->Gradient_i(x,i)*g->Gradient_i(x,j);
}

std::string Compose_SF_VF_Function::Label() const
{
  std::string sf=f->Label(), sg=g->Label();
  std::string str=sf;
  str+="(";
  str+=sg;
  str+="(x))";
  return str;
}

void Compose_SF_VF_Function::PreEval(const Vector& x)
{
  gx.resize(g->NumDimensions());
  g->PreEval(x);
  g->Eval(x,gx);
  f->PreEval(gx);
}

void Compose_SF_VF_Function::Gradient(const Vector& x,Vector& grad)
{
  Jg.resize(gx.n,x.n);
  gradf.resize(gx.n);
  g->Jacobian(x,Jg);
  f->Gradient(gx,gradf);
  Jg.mulTranspose(gradf,grad);
}

Real Compose_SF_VF_Function::Gradient_i(const Vector& x,int i)
{
  Vector Jj(gx.n);
  g->Jacobian_j(x,i,Jj);
  f->Gradient(gx,gradf);
  return Jj.dot(gradf);
}

void Compose_SF_VF_Function::Hessian(const Vector& x,Matrix& H)
{
  //f''*g'*g'^t + f'*g''
  f->Gradient(gx,gradf);
  g->Jacobian(x,Jg);
  Matrix Hf(gx.n,gx.n);
  Matrix Hgi(x.n,x.n);
  Matrix temp;

  //first term
  f->Hessian(gx,Hf);
  temp.mul(Jg,Hf);
  H.mulTransposeB(temp,Jg);

  //second term
  for(int i=0;i<x.n;i++) {
    g->Hessian_i(x,i,Hgi);
    Vector Hgit_gradf;
    Hgi.mulTranspose(gradf,Hgit_gradf);
    for(int j=0;j<x.n;i++) {
      H(i,j) += Hgit_gradf(j);
    }
  }
}

Real Compose_SF_VF_Function::Hessian_ij(const Vector& x,int i,int j)
{
  cout<<"Hessian_ij: this is totally inefficient!!!"<<endl;
  AssertNotReached();
  return 0;
}

 

std::string Compose_VF_VF_Function::Label() const
{
  std::string sf=f->Label(), sg=g->Label();
  std::string str=sf;
  str+="(";
  str+=sg;
  str+="(x))";
  return str;
}

void Compose_VF_VF_Function::PreEval(const Vector& x)
{
  gx.resize(g->NumDimensions());
  g->PreEval(x);
  g->Eval(x,gx);
  f->PreEval(gx);
}

void Compose_VF_VF_Function::Jacobian(const Vector& x,Matrix& J)
{
  g->Jacobian(x,Jg);
  f->Jacobian(gx,Jf);
  J.mul(Jf,Jg);
}

void Compose_VF_VF_Function::Jacobian_i(const Vector& x,int i,Vector& Ji)
{
  Vector Jfi(gx.n);
  g->Jacobian(x,Jg);
  f->Jacobian_i(gx,i,Jfi);
  Jg.mulTranspose(Jfi,Ji);
}

void Compose_VF_VF_Function::Jacobian_j(const Vector& x,int j,Vector& Jj)
{
  Vector Jgj(gx.n);
  g->Jacobian_j(x,j,Jgj);
  f->Jacobian(gx,Jf);
  Jf.mul(Jgj,Jj);
}



ScalarFieldDirectionalFunction::ScalarFieldDirectionalFunction(ScalarFieldFunction* _f,const Vector& _x,const Vector& _n)
  :f(_f),x(_x),n(_n)
{
  tmp.resize(x.n);
}

std::string ScalarFieldDirectionalFunction::Label() const
{
  std::string sf=f->Label();
  std::string str=sf; str+="(x+tn)";
  return str;
}

void ScalarFieldDirectionalFunction::PreEval(Real t)
{
  tmp=x; tmp.madd(n,t);
  f->PreEval(tmp);
}

Real ScalarFieldDirectionalFunction::Eval(Real t)
{
  return f->Eval(tmp);
}

Real ScalarFieldDirectionalFunction::Deriv(Real t)
{
  return f->DirectionalDeriv(tmp,n);
}

Real ScalarFieldDirectionalFunction::Deriv2(Real t)
{
  return f->DirectionalDeriv2(tmp,n);
}

ScalarFieldProjectionFunction::ScalarFieldProjectionFunction(ScalarFieldFunction* _f,const Vector& _x,int _i)
  :f(_f),x(_x),i(_i)
{
  tmp = x;
}

std::string ScalarFieldProjectionFunction::Label() const
{
  char buf[32];
  sprintf(buf,"(x+e%d)",i);
  std::string sf=f->Label();
  std::string str=sf; str+=buf; 
  return str;
}

void ScalarFieldProjectionFunction::PreEval(Real t)
{
  tmp[i] = x[i]+t;
}

Real ScalarFieldProjectionFunction::Eval(Real t) 
{
  return f->Eval(tmp);
}

Real ScalarFieldProjectionFunction::Deriv(Real t)
{
  return f->Gradient_i(tmp,i);
}

Real ScalarFieldProjectionFunction::Deriv2(Real t)
{
  return f->Hessian_ij(tmp,i,i);
}





VectorFieldProjectionFunction::VectorFieldProjectionFunction(VectorFieldFunction* _f,int _i)
  :f(_f),i(_i)
{}

std::string VectorFieldProjectionFunction::Label() const
{
  char buf[32];
  sprintf(buf,"(x+e%d)",i);
  std::string sf=f->Label();
  std::string str=sf; str+=buf; 
  return str;
}
 
Real VectorFieldProjectionFunction::Eval(const Vector& x)
{
  return f->Eval_i(x,i);
}

void VectorFieldProjectionFunction::Gradient(const Vector& x,Vector& grad)
{
  f->Jacobian_i(x,i,grad);
}




std::string ComponentVectorFieldFunction::Label() const
{
  std::string str="Components(";
  for(size_t i=0;i<functions.size();i++) {
    str+=functions[i]->Label();
    if(i+1 < functions.size())
      str+=",";
  }
  str+=")";
  return str;
}

std::string ComponentVectorFieldFunction::Label(int i) const
{
  return functions[i]->Label();
}

int ComponentVectorFieldFunction::NumDimensions() const
{
  return (int)functions.size();
}

void ComponentVectorFieldFunction::PreEval(const Vector& x)
{
  for(size_t i=0;i<functions.size();i++) {
    functions[i]->PreEval(x);
  }
}

void ComponentVectorFieldFunction::Eval(const Vector& x, Vector& v)
{
  for(size_t i=0;i<functions.size();i++)
    v(i) = functions[i]->Eval(x);
}

Real ComponentVectorFieldFunction::Eval_i(const Vector& x,int i)
{
  return functions[i]->Eval(x);
}

void ComponentVectorFieldFunction::Jacobian(const Vector& x,Matrix& J)
{
  J.resize((int)functions.size(),x.n);
  Vector vtemp;
  for(size_t i=0;i<functions.size();i++) {
    J.getRowRef(i,vtemp);
    functions[i]->Gradient(x,vtemp);
  }
}

void ComponentVectorFieldFunction::Jacobian_i(const Vector& x,int i,Vector& Ji)
{
  functions[i]->Gradient(x,Ji);
}

void ComponentVectorFieldFunction::Hessian_i(const Vector& x,int i,Matrix& Hi)
{
  functions[i]->Hessian(x,Hi);
}

void ComponentVectorFieldFunction::DirectionalDeriv(const Vector& x,const Vector& h,Vector& v)
{
  for(size_t i=0;i<functions.size();i++)
    v(i) = functions[i]->DirectionalDeriv(x,h);
}





std::string CompositeVectorFieldFunction::Label() const
{
  std::string str="Compose(";
  for(size_t i=0;i<functions.size();i++) {
    str+=functions[i]->Label();
    if(i+1 < functions.size())
      str+=",";
  }
  str+=")";
  return str;
}

std::string CompositeVectorFieldFunction::Label(int i) const
{
  int f=GetFunction(i);
  return functions[f]->Label(i);
}

int CompositeVectorFieldFunction::NumDimensions() const
{
  int nd=0;
  for(size_t i=0;i<functions.size();i++) {
    nd+=functions[i]->NumDimensions();
  }
  return nd;
}

void CompositeVectorFieldFunction::PreEval(const Vector& x)
{
  for(size_t i=0;i<functions.size();i++) {
    functions[i]->PreEval(x);
  }
}

void CompositeVectorFieldFunction::Eval(const Vector& x, Vector& v)
{
  Vector vtemp;
  int nd=0;
  for(size_t i=0;i<functions.size();i++) {
    int idims=functions[i]->NumDimensions();
    vtemp.setRef(v,nd,1,idims);
    functions[i]->Eval(x,vtemp);
    assert(vtemp.n==idims);
    nd += idims;
  }
  assert(nd == v.n);
}

Real CompositeVectorFieldFunction::Eval_i(const Vector& x,int i)
{
  int f= GetFunction(i);
  return functions[f]->Eval_i(x,i);
}

void CompositeVectorFieldFunction::Jacobian(const Vector& x,Matrix& J)
{
  J.resize(NumDimensions(),x.n);
  Matrix mtemp;
  int offset=0;
  for(size_t i=0;i<functions.size();i++) {
    mtemp.setRef(J,offset,0,1,1,functions[i]->NumDimensions(),x.n);
    functions[i]->Jacobian(x,mtemp);
    offset += mtemp.m;
  }
}

void CompositeVectorFieldFunction::Jacobian_i(const Vector& x,int i,Vector& Ji)
{
  int f=GetFunction(i);
  functions[f]->Jacobian_i(x,i,Ji);
}

void CompositeVectorFieldFunction::Hessian_i(const Vector& x,int i,Matrix& Hi)
{
  int f=GetFunction(i);
  functions[f]->Hessian_i(x,i,Hi);
}

void CompositeVectorFieldFunction::DirectionalDeriv(const Vector& x,const Vector& h,Vector& v)
{
  Vector vtemp;
  int nd=0;
  for(size_t i=0;i<functions.size();i++) {
    vtemp.setRef(v,nd,1,functions[i]->NumDimensions());
    functions[i]->DirectionalDeriv(x,h,vtemp);
    nd+=vtemp.n;
  }
  assert(v.n == nd);
}

int CompositeVectorFieldFunction::GetFunction(int& i) const
{
  int iorig = i;
  for(size_t k=0;k<functions.size();k++) {
    int nd=functions[k]->NumDimensions();
    if(i < nd) {
      return (int)k;
    }
    else {
      i -= nd;
    }
  }
  cout<<"Shouldn't ever get here!  i="<<iorig<<" must be out of range 0->"<<NumDimensions()<<endl;
  AssertNotReached();
  return -1;
}
