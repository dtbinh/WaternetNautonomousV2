
class Voxel 
{
	private:
		int pos_x, pos_y;
		float mean, variance;
		std::vector<3DPVector> points;
	
	public:
		Voxel(float x_, float y_);

		std::vector<3DPVector> getPoints();

		void add(float x_, float y_, float z_);
};

Voxel::Voxel(float x_, float y_) 
{
	pos_x = x_;
	pos_y = y_;
}

std::vector<3DPVector> Voxel::getPoints()
{
	return points;
}

void Voxel::add(float x_, float y_, float z_)
{
	3DPVector* v = new 3DPVector(x_,y_,z_);
	points.push_back(*v);
}

void Voxel::calculateMean()
{
	mean = 0;
	for (int i = 0; i < points.size(); i++)
	{
		mean += points[i].getZ();
	}
	mean = mean/points.size();
}

void Voxel::calculateVariance()
{
	variance = 0;
	for (int i = 0; i < points.size(); i++)
	{
		variance += (points[i].getZ() - mean) * (points[i].getZ() - mean);
	}
	variance = variance/points.size();
}

float Voxel::getMean()
{
	return mean;
}

float Voxel::getVariance()
{
	return variance;
}
