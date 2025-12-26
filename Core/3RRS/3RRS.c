/*
 * 3-RRS 逆运动学（yaw = 0，不绕定平台 z 自转）
 * 输入：H(动平台中心高度, m), X角(roll, deg), Y角(pitch, deg)
 * 输出：theta1, theta2, theta3（主动转角, rad/deg）
 *
 * 结构约定（按你描述写死）：
 * 1) 定平台 A1/A2/A3：120° 等边分布，A1 在 +x 轴上（角度0°）
 * 2) 动平台 C1/C2/C3：120° 分布，C1 在动平台局部 +x' 轴上（角度0°）
 * 3) yaw = 0：旋转矩阵 R = Ry(Y) * Rx(X)
 * 4) “向外突”：每条链的径向 u_i 取 O->A_i 的单位向量（外向）
 *    且定义 theta_i=0 时主动杆 AiBi 水平且沿 +u_i（向外伸）
 * 5) 主动杆长 L1，从动杆长 L2
 * 6) 角度限位 [THETA_MIN, THETA_MAX]
 *
 * 数学核心：
 *   di = Ci(base) - Ai(base)
 *   Fi = L1 * (di · zhat) = L1 * di_z
 *   Gi = L1 * (di · u_i)  （u_i 为外向径向单位向量）
 *   Hi = (||di||^2 + L1^2 - L2^2)/2
 *   解：Fi*sinθ + Gi*cosθ = Hi  -> θ 两解（±）
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include "3RRS.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* =========================
 * 参数（全部 define）
 * ========================= */
#define L1_ACT          0.08    /* 主动件长度 |AiBi| (m) */
#define L2_PAS          0.10    /* 从动件长度 |BiCi| (m) */

#define RB_BASE         0.09    /* 定平台半径（O->Ai）(m) */
#define RP_PLAT         0.12    /* 动平台半径（P->Ci，在平台局部）(m) */

#define BASE_PHASE_DEG   0.0    /* A1 在 +x 轴上：0° */
#define PLAT_PHASE_DEG   0.0    /* C1 在 +x' 轴上：0°（确保 X=Y=0 时 C1 投影在 x 轴） */

#define THETA0_RAD       0.0    /* 初始角：主动件水平（theta=0 对应水平沿 +u_i） */

#define THETA_MIN_DEG   (-60.0) /* 主动角下限（deg） */
#define THETA_MAX_DEG   ( 60.0) /* 主动角上限（deg） */

#define DEG2RAD(d) ((d) * (M_PI / 180.0))
#define RAD2DEG(r) ((r) * (180.0 / M_PI))
#define EPS 1e-12

typedef struct { double x, y, z; } Vec3;
static inline Vec3 v3(double x,double y,double z){ Vec3 v={x,y,z}; return v; }
static inline Vec3 add(Vec3 a, Vec3 b){ return v3(a.x+b.x,a.y+b.y,a.z+b.z); }
static inline Vec3 sub(Vec3 a, Vec3 b){ return v3(a.x-b.x,a.y-b.y,a.z-b.z); }
static inline double dot(Vec3 a, Vec3 b){ return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline double norm2(Vec3 a){ return dot(a,a); }
static inline double clamp(double x,double lo,double hi){ return (x<lo)?lo:((x>hi)?hi:x); }

static inline Vec3 normalize_xy(Vec3 a){
    double n = sqrt(a.x*a.x + a.y*a.y);
    if(n < EPS) return v3(1,0,0);
    return v3(a.x/n, a.y/n, 0.0);
}

static inline double wrap_pi(double a){
    while(a >  M_PI) a -= 2.0*M_PI;
    while(a < -M_PI) a += 2.0*M_PI;
    return a;
}

/* yaw=0：R = Ry(Y) * Rx(X) */
static Vec3 rotate_no_yaw(Vec3 p_local, double X_roll, double Y_pitch){
    double cx = cos(X_roll), sx = sin(X_roll);
    double cy = cos(Y_pitch), sy = sin(Y_pitch);

    /* Rx(X) */
    Vec3 t = v3(
        p_local.x,
        cx*p_local.y - sx*p_local.z,
        sx*p_local.y + cx*p_local.z
    );
    /* Ry(Y) */
    return v3(
        cy*t.x + sy*t.z,
        t.y,
        -sy*t.x + cy*t.z
    );
}

/* 解 Fi*sinθ + Gi*cosθ = Hi 的两解；不可达返回 false */
static bool solve_trig_linear(double Fi, double Gi, double Hi, double sol[2]){
    double disc = Fi*Fi + Gi*Gi - Hi*Hi;
    if(disc < -1e-10) return false;
    if(disc < 0) disc = 0;

    double sdisc = sqrt(disc);
    double denom = (Hi - Gi);

    if(fabs(denom) < EPS){
        /* 备用解法（避免半角分母病态） */
        double R = sqrt(Fi*Fi + Gi*Gi);
        if(R < EPS) return false;
        double phi = atan2(Gi, Fi);
        double s = clamp(Hi / R, -1.0, 1.0);
        double a = asin(s);
        sol[0] = wrap_pi(a - phi);
        sol[1] = wrap_pi((M_PI - a) - phi);
        return true;
    }

    sol[0] = wrap_pi(2.0 * atan2((Fi + sdisc), denom));
    sol[1] = wrap_pi(2.0 * atan2((Fi - sdisc), denom));
    return true;
}

/* 构造等边分布点：A1 在 +x；C1 在 +x' */
static void build_geometry(Vec3 A[3], Vec3 C_local[3]){
    double base_phase = DEG2RAD(BASE_PHASE_DEG);
    double plat_phase = DEG2RAD(PLAT_PHASE_DEG);

    for(int i=0;i<3;i++){
        double angA = base_phase + i*(2.0*M_PI/3.0);
        A[i] = v3(RB_BASE*cos(angA), RB_BASE*sin(angA), 0.0);

        double angC = plat_phase + i*(2.0*M_PI/3.0);
        C_local[i] = v3(RP_PLAT*cos(angC), RP_PLAT*sin(angC), 0.0);
    }
}

/*
 * 逆解：输入 H, X(roll), Y(pitch) -> theta[3]
 * prev_theta 可传 NULL；若你做轨迹仿真想连续选解，可以自己传上一时刻 theta
 */
unsigned char ik_3rrs_no_yaw(double H, double X_roll, double Y_pitch,
                    const double* prev_theta /*可为空*/,
                    double theta_out[3])
{
    Vec3 A[3], C_local[3];
    build_geometry(A, C_local);

    Vec3 P = v3(0,0,H);          /* 动平台中心：仅沿 z 平移高度 H */
    Vec3 zhat = v3(0,0,1);

    double th_min = DEG2RAD(THETA_MIN_DEG);
    double th_max = DEG2RAD(THETA_MAX_DEG);

    for(int i=0;i<3;i++){
        /* Ci(base) = P + R * Ci_local */
        Vec3 Ci = add(P, rotate_no_yaw(C_local[i], X_roll, Y_pitch));
        Vec3 di = sub(Ci, A[i]);

        /* 外向径向 u_i = normalize(O->Ai)  —— “向外突”的约定在这里体现 */
        Vec3 ui = normalize_xy(A[i]);

        double Fi = L1_ACT * dot(di, zhat);
        double Gi = L1_ACT * dot(di, ui);
        double Hi = (norm2(di) + L1_ACT*L1_ACT - L2_PAS*L2_PAS) / 2.0;

        double cand[2];
        if(!solve_trig_linear(Fi, Gi, Hi, cand)) return false;

        /* 加上“初始角水平”的偏置（默认 0） */
        cand[0] = wrap_pi(cand[0] + THETA0_RAD);
        cand[1] = wrap_pi(cand[1] + THETA0_RAD);

        /* 限位过滤 */
        bool ok0 = (cand[0] >= th_min - 1e-9 && cand[0] <= th_max + 1e-9);
        bool ok1 = (cand[1] >= th_min - 1e-9 && cand[1] <= th_max + 1e-9);
        if(!ok0 && !ok1) return false;

        /* 选解策略：
         * - 若有 prev_theta：选离上一时刻最近的合法解（连续性）
         * - 否则：选幅值更小的合法解
         */
        double chosen;
        if(prev_theta){
            double best = 1e100;
            if(ok0){
                double d0 = fabs(wrap_pi(cand[0] - prev_theta[i]));
                if(d0 < best){ best = d0; chosen = cand[0]; }
            }
            if(ok1){
                double d1 = fabs(wrap_pi(cand[1] - prev_theta[i]));
                if(d1 < best){ best = d1; chosen = cand[1]; }
            }
        }else{
            double a0 = ok0 ? fabs(wrap_pi(cand[0])) : 1e100;
            double a1 = ok1 ? fabs(wrap_pi(cand[1])) : 1e100;
            chosen = (a0 <= a1) ? cand[0] : cand[1];
        }

        theta_out[i] = clamp(chosen, th_min, th_max);
    }
    return true;
}

/* ====== 示例主函数：输入 H、X角、Y角，输出三主动角 ====== 
int main(void){
    double H, Xdeg, Ydeg;
    printf("输入 H(m)  X角(roll,deg)  Y角(pitch,deg)：\n");
    if(scanf("%lf %lf %lf", &H, &Xdeg, &Ydeg) != 3){
        printf("输入格式错误。\n");
        return 0;
    }

    double X = DEG2RAD(Xdeg);
    double Y = DEG2RAD(Ydeg);

    double theta[3];
    if(!ik_3rrs_no_yaw(H, X, Y, NULL, theta)){
        printf("该姿态不可达或超出关节限位。\n");
        return 0;
    }

    printf("输出主动角：\n");
    for(int i=0;i<3;i++){
        printf("theta%d = %.8f rad  (%.4f deg)\n", i+1, theta[i], RAD2DEG(theta[i]));
    }
    return 0;
}
*/

