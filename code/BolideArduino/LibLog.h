#if TELEMETRY
template<typename T>
void println(T str)
{
  ;
}

template<typename T>
void print(T str)
{
  ;
}

void print()
{
  ;
}

void println()
{
  ;
}
#else
template<typename T>
void println(T str){
  Serial.println(str);
}

template<typename T>
void print(T str){
  Serial.print(str);
}



void println(){
  Serial.println();
}
#endif
