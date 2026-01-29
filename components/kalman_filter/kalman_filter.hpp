#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

// Class definitions for Kalman filters
class GPSKalmanFilter
{
private:
    // Tham số bộ lọc
    float _measurementUncertainty; // Độ không chắc chắn đo lường
    float _estimationUncertainty;  // Độ không chắc chắn ước lượng
    float _processNoise;           // Nhiễu quá trình

    // Trạng thái hiện tại
    float _latEstimate;    // Ước lượng vĩ độ
    float _lngEstimate;    // Ước lượng kinh độ
    float _latUncertainty; // Độ không chắc chắn vĩ độ
    float _lngUncertainty; // Độ không chắc chắn kinh độ
    bool _initialized;     // Đã khởi tạo chưa

public:
    // Hàm khởi tạo với các tham số mặc định
    GPSKalmanFilter(float measurementUncertainty = 0.1,
                    float estimationUncertainty = 1.0,
                    float processNoise = 0.01)
        : _measurementUncertainty(measurementUncertainty),
          _estimationUncertainty(estimationUncertainty),
          _processNoise(processNoise),
          _latEstimate(0.0),
          _lngEstimate(0.0),
          _latUncertainty(estimationUncertainty),
          _lngUncertainty(estimationUncertainty),
          _initialized(false) {}

    // Thiết lập lại bộ lọc
    void reset(float initialLat = 0.0, float initialLng = 0.0)
    {
        _latEstimate = initialLat;
        _lngEstimate = initialLng;
        _latUncertainty = _estimationUncertainty;
        _lngUncertainty = _estimationUncertainty;
        _initialized = (initialLat != 0.0 || initialLng != 0.0);
    }

    // Cập nhật bộ lọc với dữ liệu mới
    void updatePosition(float measuredLat, float measuredLng, float *filteredLat, float *filteredLng)
    {
        // Nếu chưa khởi tạo, sử dụng vị trí đầu tiên như là điểm xuất phát
        if (!_initialized && (measuredLat != 0.0 || measuredLng != 0.0))
        {
            _latEstimate = measuredLat;
            _lngEstimate = measuredLng;
            _initialized = true;
        }

        if (_initialized)
        {
            // Cập nhật vĩ độ
            _latUncertainty += _processNoise;
            float kalmanGainLat = _latUncertainty / (_latUncertainty + _measurementUncertainty);
            _latEstimate += kalmanGainLat * (measuredLat - _latEstimate);
            _latUncertainty = (1 - kalmanGainLat) * _latUncertainty;

            // Cập nhật kinh độ
            _lngUncertainty += _processNoise;
            float kalmanGainLng = _lngUncertainty / (_lngUncertainty + _measurementUncertainty);
            _lngEstimate += kalmanGainLng * (measuredLng - _lngEstimate);
            _lngUncertainty = (1 - kalmanGainLng) * _lngUncertainty;
        }

        // Trả về kết quả đã lọc
        *filteredLat = _latEstimate;
        *filteredLng = _lngEstimate;
    }
};

// Bộ lọc Kalman cho MPU6050 chỉ lọc gia tốc, không lọc tốc độ góc
class MPUKalmanFilter
{
private:
    // Tham số bộ lọc
    float _measurementUncertainty; // Độ không chắc chắn đo lường
    float _estimationUncertainty;  // Độ không chắc chắn ước lượng
    float _processNoise;           // Nhiễu quá trình

    // Trạng thái hiện tại
    float _xEstimate;    // Ước lượng gia tốc trục x
    float _yEstimate;    // Ước lượng gia tốc trục y
    float _zEstimate;    // Ước lượng gia tốc trục z
    float _xUncertainty; // Độ không chắc chắn trục x
    float _yUncertainty; // Độ không chắc chắn trục y
    float _zUncertainty; // Độ không chắc chắn trục z
    bool _initialized;   // Đã khởi tạo chưa

public:
    // Hàm khởi tạo với các tham số mặc định
    MPUKalmanFilter(float measurementUncertainty = 0.5,
                    float estimationUncertainty = 1.0,
                    float processNoise = 0.05)
        : _measurementUncertainty(measurementUncertainty),
          _estimationUncertainty(estimationUncertainty),
          _processNoise(processNoise),
          _xEstimate(0.0),
          _yEstimate(0.0),
          _zEstimate(9.8), // Gia tốc trọng trường
          _xUncertainty(estimationUncertainty),
          _yUncertainty(estimationUncertainty),
          _zUncertainty(estimationUncertainty),
          _initialized(false)
    {
    }

    // Thiết lập lại bộ lọc
    void reset()
    {
        _xEstimate = 0.0;
        _yEstimate = 0.0;
        _zEstimate = 9.8; // Gia tốc trọng trường
        _xUncertainty = _estimationUncertainty;
        _yUncertainty = _estimationUncertainty;
        _zUncertainty = _estimationUncertainty;
        _initialized = false;
    }

    // Cập nhật bộ lọc với dữ liệu mới
    void updateAcceleration(float measuredX, float measuredY, float measuredZ,
                            float *filteredX, float *filteredY, float *filteredZ)
    {
        // Nếu chưa khởi tạo, sử dụng giá trị đầu tiên
        if (!_initialized)
        {
            _xEstimate = measuredX;
            _yEstimate = measuredY;
            _zEstimate = measuredZ;
            _initialized = true;
        }

        if (_initialized)
        {
            // Cập nhật trục x
            _xUncertainty += _processNoise;
            float kalmanGainX = _xUncertainty / (_xUncertainty + _measurementUncertainty);
            _xEstimate += kalmanGainX * (measuredX - _xEstimate);
            _xUncertainty = (1 - kalmanGainX) * _xUncertainty;

            // Cập nhật trục y
            _yUncertainty += _processNoise;
            float kalmanGainY = _yUncertainty / (_yUncertainty + _measurementUncertainty);
            _yEstimate += kalmanGainY * (measuredY - _yEstimate);
            _yUncertainty = (1 - kalmanGainY) * _yUncertainty;

            // Cập nhật trục z
            _zUncertainty += _processNoise;
            float kalmanGainZ = _zUncertainty / (_zUncertainty + _measurementUncertainty);
            _zEstimate += kalmanGainZ * (measuredZ - _zEstimate);
            _zUncertainty = (1 - kalmanGainZ) * _zUncertainty;
        }

        // Trả về kết quả đã lọc
        *filteredX = _xEstimate;
        *filteredY = _yEstimate;
        *filteredZ = _zEstimate;
    }
};

#endif // KALMAN_FILTER_H