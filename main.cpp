#include <iostream>
#include <array>
#include <random>
#include "miniwindow.h"

template<typename T>
{
	T x, y, z, w;
};
template<typename T> Vector4<T> operator+(Vector4<T> const& v1, Vector4<T> const& v2){ return {v1.x + v2.x, v1.y +v2.y, v1.z+v2.z, v1.w+v2.w}; }
template<typename T> Vector4<T> operator-(Vector4<T> const& v1, Vector4<T> const& v2){ return {v1.x - v2.x, v1.y - v2.y, v1.z-v2.z, v1.w-v2.w}; }
template<typename T> Vector4<T> operator*(T const& scl, Vector4<T> const& v){ return {scl*v.x, scl*v.y, scl*v.z, scl*v.w}; }
template<typename T> Vector4<T> operator*(Vector4<T> const& v, T const& scl){ return {v.x*scl, v.y*scl, v.z*scl, v.w*scl}; }
template<typename T> Vector4<T> operator/(Vector4<T> const& v, T const& scl){ return {v.x/scl, v.y/scl, v.z/scl, v.w/scl}; }

template<typename State, typename T, typename RHS, typename Callback> 
auto solve_rk4(State y0, T t0, T t1, T h, RHS f, Callback cb)
{
	T t = t0;
	State y = y0;
	while(t < t1)
	{
		if(t + h > t1){ h = t1 - t; }
		State k1 = f(t, y);
		State k2 = f(t + h * (T)0.5, y + (h * (T)0.5) * k1);
		State k3 = f(t + h * (T)0.5, y + (h * (T)0.5) * k2);
		State k4 = f(t + h,          y +  h           * k3);
	
        y = y + (k1 + k4 + (T)2 * (k2 + k3)) * (h / (T)6);
		t = t + h;
		cb(t, y);
	}
	return y;
}

struct App
{
	MainWindow wnd;

	Vector4<double> state;
	double time, h;
	double a, b, c, d;

	std::vector<Vector4<double>> data;

	App()
	{
		a = 20., b = 1., c = 30.0, d = 1.;
		h = 0.00002;
		time = 0.0;
		state = {8.0, 12.0};
	}

	int enterApp()
	{
		wnd.window.eventDriven = false;

		wnd.mouseHandler([&](Mouse const&){ });
		wnd.resizeHandler([&](int /*w*/, int /*h*/, StateChange /*sc*/){} );
		wnd.idleHandler([&]
		{
			state = solve_rk4(state, time, time + 0.001, h,
			[this](double const& /*t*/, Vector4<double> const& s)->Vector4<double>
			{
				double den=2*m1 + m2 - m2* cos(2*s.x-2*s.y);
				double a = (-g*(2*m1+m2)*sin(s.x)-m2*g*sin(s.x-2*s.y)-2*sin(s.x-s.y)*m2*(s.w*s.w*L2+s.z*s.z*L1*cos(s.x-s.y))) / (L1*den);
				double b = (2*sin(s.x-s.y)*(s.z*s.z*L1*(m1+m2)+g*(m1+m2)*cos(s.x)+s.z*s.z*L2*m2*cos(s.x-s.y)))/(L2*den);
				return {s.z, s.w, a, b};
			},
			[this](double const& /*t*/, Vector4<double> const& state)
			{
				data.push_back(state);
			} );
		});
		wnd.exitHandler([&]{});

		wnd.renderHandler( [&](SoftwareRenderer& r)
		{
			r.forall_pixels([](auto, auto, auto){ return color(255, 255, 255); });
			auto blue = [](auto){return color(10, 0, 167);};
			auto black = [](auto) {return color(0, 0, 0);};
			auto red = [](auto) {return color(240, 0, 21);};

			if(data.size() > 4)
			{
			double L1s=70;
			double L2s=70;
			double x1= 300+L1s*cos(([&](double i){ return data[(int)i].x;}));
			double y1= 200+L1s*sin(([&](double i){ return data[(int)i].x;}));
			double x2= x1+L2s*cos(([&](double i){ return data[(int)i].y;}));
			double y2= y1 + L2s*sin(([&](double i){ return data[(int)i].y;}));

			r.ellipse(300, 200, 5, 5, black);
			r.line(300, 200, x1, y1, blue);
			r.ellipse(x1, y1, 5, 5, blue);
			r.line(x1, y1, x2, y2, red);
			r.ellipse(x2, y2, 5, 5, red);
			}
		});

		bool res = wnd.open(L"Double Pendulum Simulation", {64, 64}, {640, 480}, true, [&]{ return true; });
		return res ? 0 : -1;
	}
};
//#ifdef _WIN32
//int WINAPI wWinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, PWSTR pCmdLine, int nCmdShow)
//#else
int main()
//#endif
{
	return App{}.enterApp();
}
