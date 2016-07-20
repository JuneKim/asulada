#ifndef __DATA_H__

namespace Asulada {
class Pos {
public:
	Pos(int x, int y, int z) : x_(x), y_(y), z_(z) {}
	virtual ~Pos();
	void getPos(int& x, int& y, int& z)
	{
		x = x_;
		y = y_;
		z = z_;
	}

private:
	int x_, y_, z_;
}; // class Pos
} // namespace Asulada

#endif
