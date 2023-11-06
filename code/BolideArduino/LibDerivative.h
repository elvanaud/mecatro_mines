#ifndef LIB_DER_
#define LIB_DER_

//#define BUF_SIZE 40 //todo: set that as a parameter because right now I use only two values for derivatives, and 40 for mean 

class Buffer
{
public:
  Buffer(int bufSize): BUF_SIZE(bufSize){
    buf = new double[BUF_SIZE];
    for(int i = 0; i < BUF_SIZE; i++){
      buf[i] = 0;
    }
  }
  ~Buffer(){
    delete buf;
  }
  void push(double val){
    buf[idxLast] = val;
    idxLast++;
    idxLast %= BUF_SIZE;
    bufSize++;
  }

  double last(){
    /*if(bufSize == 0){
      throw "Error: buffer empty";
    }*/
    //assert(bufSize != 0);
    return buf[idxLast];
  }

  double beforeLast(){
    /*if(bufSize < 2){
      throw "Error: not enough values in buffer";
    }*/
    //assert(bufSize >= 2);
    //int idx = idxLast;
    if(idxLast == 0){
      return buf[BUF_SIZE-1];
    }
    return buf[idxLast-1];
  }

  double derivative(double dt)
  {
    return (last() - beforeLast())/dt;
  }

  double mean()
  {
    double sum = 0;
    for(int i = 0; i < BUF_SIZE; i++){ //the first BUF_SIZE steps will have "wrong" values
      sum += buf[i];
    }
    return sum / BUF_SIZE;
  }
private:
  const int BUF_SIZE = 10;
  double *buf;
  int bufSize = 0; //Used while buffer is loading
  int idxLast = 0;
};

//class Integrator{};

#endif

