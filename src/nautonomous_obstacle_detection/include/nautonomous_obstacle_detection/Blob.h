
class Blob 
{
	private:
		int size;
		double Cornerpoints[8];
		std::vector<PVector> points;
	
	public:
		Blob(float x_, float y_);

		int getSize();

		std::vector<PVector> getPoints();

		void add(float x_, float y_);
	
		bool isClose( float x_, float y_);
};

Blob::Blob(float x_, float y_) 
{
	PVector* v = new PVector(x_,y_);
	points.push_back(*v);
	size = 1;
}

int Blob::getSize()
{
	return size;
}

std::vector<PVector> Blob::getPoints()
{
	return points;
}

void Blob::add(float x_, float y_)
{
	PVector* v = new PVector(x_,y_);
	points.push_back(*v);
	size++;
}

bool Blob::isClose( float x_, float y_)
{
	bool found = false;
	for ( int i = 0; i < size; i++)
	{
		float d = (points.at(i).getX() - x_) * (points.at(i).getX() - x_) + (points.at(i).getY() - y_) * (points.at(i).getY() - y_); 
		if (d < 25)
		{
			found = true;
			break;
		}
		else
		{
			found = false;
		}
	} 
	return found;		
}
