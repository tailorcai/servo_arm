#pragma once

class MeanFilter {
  protected:
    int n_items, size;
    int *items;
  public:
  MeanFilter(int samples):size(samples), n_items(0) {
    items = new int[samples];
  }

  void push(int v) {
    if( n_items == size) 
    {
      memmove( items, items+1, sizeof(int)*(size-1));
      n_items--;
    }

    items[n_items++] = v;
  }

  int mean() {
    long sum = 0;
    if( n_items ) {
      for( int i=0;i<n_items;i++)
        sum += items[i];
      return sum / n_items;
    }
    return 0;
  }
};

class SerialHelper {
public:
  static int readline(int readch, char *buffer, int len) {
    static int pos = 0;
    int rpos;

    if (readch > 0) {
        switch (readch) {
            case '\r': // Ignore CR
                break;
            case '\n': // Return on new-line
                rpos = pos;
                pos = 0;  // Reset position index ready for next time
                return rpos;
            default:
                if (pos < len-1) {
                    buffer[pos++] = readch;
                    buffer[pos] = 0;
                }
        }
    }
    return 0;
  }

  // return number of chars processed
  static int parse_next(char* in_buf, char* out_buf) 
  {
    // copy to next separate
    int len = 0;
    while( *in_buf != ' ' || *in_buf != 0 || *in_buf != '\n' || *in_buf != '\r') {
      *out_buf = *in_buf;
      out_buf ++;
      in_buf ++;
      len ++;
    }
    if( *in_buf == ' ' )  // skip separate
      len ++;
    *out_buf = '\0';  // end
    return len;
  }
};
