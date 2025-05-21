#include <iostream>
#include <vector>

struct Point {
    double x, y;
    Point(double x = 0, double y = 0) : x(x), y(y) {}
};

// 计算向量叉积 (B-A) × (C-A)
double ccw(const Point& A, const Point& B, const Point& C) {
    return (B.x - A.x) * (C.y - A.y) - (B.y - A.y) * (C.x - A.x);
}

// 判断点P是否在线段AB上
bool isPointOnSegment(const Point& P, const Point& A, const Point& B) {
    return (std::min(A.x, B.x) <= P.x + 1e-9 && P.x <= std::max(A.x, B.x) + 1e-9) &&
           (std::min(A.y, B.y) <= P.y + 1e-9 && P.y <= std::max(A.y, B.y) + 1e-9);
}

// 判断线段AB和CD是否相交
bool segmentsIntersect(const Point& A, const Point& B, const Point& C, const Point& D) {
    // 快速排斥试验
    if (std::max(A.x, B.x) < std::min(C.x, D.x) - 1e-9 ||
        std::max(C.x, D.x) < std::min(A.x, B.x) - 1e-9 ||
        std::max(A.y, B.y) < std::min(C.y, D.y) - 1e-9 ||
        std::max(C.y, D.y) < std::min(A.y, B.y) - 1e-9) {
        return false;
    }
    
    // 跨立试验
    double ccw1 = ccw(A, B, C);
    double ccw2 = ccw(A, B, D);
    double ccw3 = ccw(C, D, A);
    double ccw4 = ccw(C, D, B);
    
    // 判断叉积符号
    if ((ccw1 * ccw2 <= 1e-9) && (ccw3 * ccw4 <= 1e-9)) {
        // 处理端点在另一条线段上的情况
        if ((std::abs(ccw1) < 1e-9 && isPointOnSegment(C, A, B)) ||
            (std::abs(ccw2) < 1e-9 && isPointOnSegment(D, A, B)) ||
            (std::abs(ccw3) < 1e-9 && isPointOnSegment(A, C, D)) ||
            (std::abs(ccw4) < 1e-9 && isPointOnSegment(B, C, D))) {
            return true;
        }
        return true;
    }
    return false;
}

void test() {
    if(true){// 测试线段相交的基本情况
        Point A(0, 0), B(1, 1), C(1, 0), D(0, 1);
        std::cout << (segmentsIntersect(A, B, C, D) ? "True" : "False") << std::endl;  // 输出: True
    }
    if(true){// 测试线段不相交的情况
         Point A(0, 0), B(1, 1), C(2, 0), D(3, 1);
        std::cout << (segmentsIntersect(A, B, C, D) ? "True" : "False") << std::endl;  // 输出: True
    }
    if(true){// 测试共线但不相交的情况
        Point A(0, 0), B(1, 0), C(2, 0), D(3, 0);
        std::cout << (segmentsIntersect(A, B, C, D) ? "True" : "False") << std::endl;  // 输出: True
    }
    if(true){// 测试共线且部分重叠的情况
        Point A(0, 0), B(2, 0), C(1, 0), D(3, 0);
        std::cout << (segmentsIntersect(A, B, C, D) ? "True" : "False") << std::endl;  // 输出: True
    }
    if(true){// 测试端点相交共线的情况
        Point A(0, 0), B(1, 1), C(1, 1), D(2, 2);
        std::cout << (segmentsIntersect(A, B, C, D) ? "True" : "False") << std::endl;  // 输出: True
    }
    if(true){// 测试平行但不相交的情况
        Point A(0, 0), B(1, 1), C(0, 1), D(1, 2);
        std::cout << (segmentsIntersect(A, B, C, D) ? "True" : "False") << std::endl;  // 输出: True
    }
    if(true){// 测试垂直线段相交
        Point A(1, 0), B(1, 2), C(0, 1), D(2, 1);
        std::cout << (segmentsIntersect(A, B, C, D) ? "True" : "False") << std::endl;  // 输出: True
    }
    if(true){// 测试一个线段完全包含在另一个线段中的情况
        Point A(0, 0), B(3, 0), C(1, 0), D(2, 0);
        std::cout << (segmentsIntersect(A, B, C, D) ? "True" : "False") << std::endl;  // 输出: True
    }
    if(true){// 测试线段共享一个端点但不共线的情况
        Point A(0, 0), B(1, 1), C(1, 1), D(1, 2);
        std::cout << (segmentsIntersect(A, B, C, D) ? "True" : "False") << std::endl;  // 输出: True
    }
    return;
}