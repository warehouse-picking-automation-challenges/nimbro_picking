#include <opencv2/opencv.hpp>


const unsigned int BINSIZE = 512;

class Histogram
{
public:
    Histogram(int size, float min_z, float max_z)
            : m_bins(size, 0)
            , m_min_z(min_z)
            , m_max_z(max_z)
            , m_count(0)
    {
    }

    void clear()
    {
        std::fill(m_bins.begin(), m_bins.end(), 0);
        m_count = 0;
    }

    unsigned int binForValue(float z)
    {
        int bin = (z - m_min_z) * m_bins.size() / (m_max_z - m_min_z);

        return std::max<int>(0, std::min<int>(m_bins.size()-1, bin));
    }

    float valueForBin(int bin)
    {
        return bin * (m_max_z - m_min_z) / m_bins.size() + m_min_z;
    }

    void add(float value)
    {
        unsigned int bin = binForValue(value);
        m_bins[bin]++;
        m_count++;
    }

    void remove(float value)
    {
        unsigned int bin = binForValue(value);
        m_bins[bin]--;
        m_count--;
    }

    void incBin(int bin)
    {
        m_bins[bin]++;
        m_count++;
    }

    void decBin(int bin)
    {
        m_bins[bin]--;
        m_count--;
    }

    float median()
    {
        unsigned int bin;
        unsigned int sum = 0;
        for(bin = 0; bin < m_bins.size(); ++bin)
        {
            sum += m_bins[bin];
            if(sum >= m_count/2)
                break;
        }

        return valueForBin(bin);
    }
    void print()
    {
        for(size_t i = 0; i < m_bins.size(); ++i)
        {
            if(m_bins[i] == 0)
                continue;

        }
    }
private:
    std::vector<unsigned int> m_bins;
    float m_min_z;
    float m_max_z;
    unsigned int m_count;
};

static float medianFromVector(std::vector<float>& v)
{
    std::size_t n = (v.size() - 1)/2;

    std::nth_element(v.begin(), v.begin()+n, v.end());

    if(v.size() % 2 == 0)
    {
        return (v[n] + *std::min_element(v.begin()+n+1, v.end())) / 2.0f;
    }
    else
        return v[n];
}

cv::Mat_<float> hybridMedianFilter(const cv::Mat_<float>& input, int windowSize)
{
    cv::Mat_<float> output(input.rows, input.cols);

    float minValue = std::numeric_limits<float>::infinity();
    float maxValue = -std::numeric_limits<float>::infinity();

    for(int y = 0; y < input.rows; ++y)
    {
        for(int x = 0; x < input.cols; ++x)
        {
            float val = input(y,x);
            if(!std::isfinite(val))
                continue;

            minValue = std::min(minValue, val);
            maxValue = std::max(maxValue, val);
        }
    }

    Histogram histogram(BINSIZE, minValue, maxValue);


    // Pre-compute depth & histogram indices
    std::vector<uint16_t> histogram_indices(input.rows*input.cols);
    for(int y = 0; y < input.rows; ++y)
    {
        for(int x = 0; x < input.cols; ++x)
        {
            float value = input(y,x);
            if(!std::isfinite(value))
                histogram_indices[y * input.cols + x] = 0xFFFF;
            else
                histogram_indices[y * input.cols + x] = histogram.binForValue(value);
        }
    }

    std::vector<float> plus;
    std::vector<float> diag;

    for(int y = 0; y < input.rows; ++y)
    {
        for(int x = 0; x < input.cols; ++x)
        {
            plus.clear();
            diag.clear();

            for(int dy = -windowSize; dy <= windowSize; ++dy)
            {
                int my = y + dy;
                if(my < 0 || my >= input.rows)
                    continue;

                float val = input(my, x);
                if(!std::isfinite(val))
                    continue;

                plus.push_back(val);
            }

            for(int dx = -windowSize; dx <= windowSize; ++dx)
            {
                int mx = x + dx;
                if(mx < 0 || mx >= input.cols)
                    continue;

                float val = input(y, mx);
                if(!std::isfinite(val))
                    continue;

                plus.push_back(val);
            }

            for(int i = -windowSize; i <= windowSize; ++i)
            {
                int mx = x + i;
                int my = y + i;

                if(my < 0 || my >= input.rows)
                    continue;

                if(mx > 0 && mx < input.cols)
                {
                    float val = input(my, mx);
                    if(std::isfinite(val))
                        diag.push_back(val);
                }

                mx = x - i;

                if(mx > 0 && mx < input.cols)
                {
                    float val = input(my, mx);
                    if(std::isfinite(val))
                        diag.push_back(val);
                }
            }

            std::vector<float> medians;
            if(!plus.empty())
                medians.push_back(medianFromVector(plus));

            if(!diag.empty())
                medians.push_back(medianFromVector(diag));

            if(std::isfinite(input(y,x)))
                medians.push_back(input(y,x));

            if(medians.size() >= 2)
                output(y,x) = medianFromVector(medians);
            else
                output(y,x) = NAN;
        }
    }

    return output;
}


cv::Mat_<float> medianFilter(const cv::Mat_<float>& input, unsigned int windowSize)
{
    cv::Mat_<float> output(input.rows, input.cols);

    float minValue = std::numeric_limits<float>::infinity();
    float maxValue = -std::numeric_limits<float>::infinity();

    for(int y = 0; y < input.rows; ++y)
    {
        for(int x = 0; x < input.cols; ++x)
        {
            float val = input(y,x);
            if(!std::isfinite(val))
                continue;

            minValue = std::min(minValue, val);
            maxValue = std::max(maxValue, val);
        }
    }

    Histogram histogram(BINSIZE, minValue, maxValue);

    // Pre-compute depth & histogram indices
    std::vector<uint16_t> histogram_indices(input.rows*input.cols);
    for(int y = 0; y < input.rows; ++y)
    {
        for(int x = 0; x < input.cols; ++x)
        {
            float value = input(y,x);
            if(!std::isfinite(value))
                histogram_indices[y * input.cols + x] = 0xFFFF;
            else
                histogram_indices[y * input.cols + x] = histogram.binForValue(value);
        }
    }

    for(int y = 0; y < input.rows; ++y)
    {
        // The histogram is updated in a sliding window fashion.
        // Init histogram on left side
        histogram.clear();

        unsigned int window_min_y = std::max<int>(0, (int)y - windowSize);
        unsigned int window_max_y = std::min<int>(input.rows-1, y + windowSize);

        for(unsigned int _y = window_min_y; _y < window_max_y; ++_y)
        {
            for(unsigned int _x = 0; _x < windowSize; ++_x)
            {
                int bin = histogram_indices[_y * input.cols + _x];
                if(bin != 0xFFFF)
                    histogram.incBin(bin);
            }
        }

        for(int x = 0; x < input.cols; ++x)
        {
            float median = histogram.median();
            output(y,x) = median;

            // Forget the left column
            if(x >= (int)windowSize)
            {
                for(unsigned int _y = window_min_y; _y < window_max_y; ++_y)
                {
                    int bin = histogram_indices[_y * input.cols + x - windowSize];
                    if(bin != 0xFFFF)
                        histogram.decBin(bin);
                }
            }

            // Add the right column
            if(x+(int)windowSize < input.cols)
            {
                for(unsigned int _y = window_min_y; _y < window_max_y; ++_y)
                {
                    int bin = histogram_indices[_y * input.cols + x + windowSize];
                    if(bin != 0xFFFF)
                        histogram.incBin(bin);
                }
            }
        }
    }

    return output;
}