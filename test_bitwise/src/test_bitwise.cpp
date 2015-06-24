#include <stdio.h>
#include <iostream>

using namespace std;

int main()
{
	cout << "Test bitwise"<<endl;
	unsigned int t = 14, d = 9 ;
	unsigned int mask1 = (0xffffffff >> 2) << 2;
	unsigned int mask2 = 0xffffffff ^ mask1;

	unsigned int m = (t&mask1)^(d&mask2);
	unsigned int k = (t&mask2)^(d&mask1);

	cout<< m<< ":"<<k<<endl;
	unsigned int xorke = t^0x0000000f;
	cout << xorke <<endl;
	unsigned int swap = (0xffffffff>>31)<<2;
	cout<<swap<<endl;
}
