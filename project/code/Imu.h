
#if IMU_MODE

// ����������ת����ԭʼ����ת����
//#define GYRO_DATA_X              (imu963ra_gyro_x)        // ������ X ��ԭʼ����
//#define GYRO_DATA_Y              (-imu963ra_gyro_y)        // ������ Y ��ԭʼ���ݣ�������ת����
//#define GYRO_DATA_Z              (-imu963ra_gyro_z)        // ������ Z ��ԭʼ���ݣ�������ת����
//#define GYRO_TRANSITION_FACTOR   (14.3f)                 // ����������ת��ϵ����LSB/(��/s)��
//
//// ���ٶȼ�����ת��
//#define ACC_DATA_X               (imu963ra_acc_x)         // ���ٶȼ� X ��ԭʼ����
//#define ACC_DATA_Y               (-imu963ra_acc_y)         // ���ٶȼ� Y ��ԭʼ���ݣ�������ת����
//#define ACC_DATA_Z               (-imu963ra_acc_z)         // ���ٶȼ� Z ��ԭʼ���ݣ�������ת����
//#define ACC_TRANSITION_FACTOR    (4098.0f)                 // ���ٶȼ�����ת��ϵ����LSB/g��


//#define GYRO_DATA_X              (imu660ra_gyro_x)        // ������ X ��ԭʼ����
//#define GYRO_DATA_Y              (-imu660ra_gyro_y)        // ������ Y ��ԭʼ���ݣ�������ת����
//#define GYRO_DATA_Z              (-imu660ra_gyro_z)        // ������ Z ��ԭʼ���ݣ�������ת����
//#define GYRO_TRANSITION_FACTOR   (16.384f)                 // ����������ת��ϵ����LSB/(��/s)��
//
//// ���ٶȼ�����ת��
//#define ACC_DATA_X               (imu660ra_acc_x)         // ���ٶȼ� X ��ԭʼ����
//#define ACC_DATA_Y               (-imu660ra_acc_y)         // ���ٶȼ� Y ��ԭʼ���ݣ�������ת����
//#define ACC_DATA_Z               (-imu660ra_acc_z)         // ���ٶȼ� Z ��ԭʼ���ݣ�������ת����
//#define ACC_TRANSITION_FACTOR    (4096.0f)                 // ���ٶȼ�����ת��ϵ����LSB/g��

#define GYRO_DATA_X              (imu660rb_gyro_x)        // ������ X ��ԭʼ����
#define GYRO_DATA_Y              (-imu660rb_gyro_y)        // ������ Y ��ԭʼ���ݣ�������ת����
#define GYRO_DATA_Z              (-imu660rb_gyro_z)        // ������ Z ��ԭʼ���ݣ�������ת����
#define GYRO_TRANSITION_FACTOR   (14.3f)                 // ����������ת��ϵ����LSB/(��/s)��

// ���ٶȼ�����ת��
#define ACC_DATA_X               (imu660rb_acc_x)         // ���ٶȼ� X ��ԭʼ����
#define ACC_DATA_Y               (-imu660rb_acc_y)         // ���ٶȼ� Y ��ԭʼ���ݣ�������ת����
#define ACC_DATA_Z               (-imu660rb_acc_z)         // ���ٶȼ� Z ��ԭʼ���ݣ�������ת����
#define ACC_TRANSITION_FACTOR    (4098.0f)                 // ���ٶȼ�����ת��ϵ����LSB/g��


#define ACC_GRAVITY               (9.80665f)               // �������ٶȲο�ֵ��m/s2��


typedef struct quaternion_data
{
    float rot_mat[3][3];                                  // ��ת����
} quaternion_data;

typedef struct quaternion_process
{
    float qua[4];                                         // ��Ԫ�����ݣ�w, x, y, z ˳��
    float acc_filtered[3];                                // �˲���ļ��ٶȣ���λ��g��
} quaternion_process;

typedef struct quaternion_parameter
{
    float acc_err[3];                                    // ���ٶȼ�У׼���ֵ
} quaternion_parameter;

typedef struct quaternion_module
{
    quaternion_process pro;                              // ��Ԫ����������
    quaternion_data data;                                // ��Ԫ������������
    quaternion_parameter parameter;                      // ��Ԫ��У׼����
} quaternion_module;

typedef struct
{
    float p;                                             // PID ������������ P
    float i;                                             // PID ������������ I
    float d;                                             // PID ������΢���� D
    float p_value_last;                                  // ��һ��ƫ��ֵ
    float i_value;                                       // PID ����ֵ
    float i_value_pro;                                   // PID ����ֵ�ı�������Χ 0 - 1���������ƻ��������ٶȣ�
    float i_value_max;                                   // PID ����ֵ����
    float out;                                           // PID ���������ֵ
    float out_max;                                       // PID ���ֵ����
    float incremental_data[2];                           // ����ʽ PID ��ƫ����ʷ����
} pid_cycle_struct;

typedef struct
{
    float correct_kp;                                    // ��̬У׼����ϵ������Χ 0.1 - 0.5��
    float correct_ki;                                    // ��̬У׼����ϵ������Χ 0.001 - 0.01��
    float call_cycle;                                    // ��̬�˲��㷨�ĵ������ڣ���λ��s���룩��
    float mechanical_zero;                               // ��е���
    float yaw;                                           // ƫ���ǣ���λ���ȣ�
    float rol;                                           // ����ǣ���λ���ȣ�
    float pit;                                           // �����ǣ���λ���ȣ�
} cascade_common_value_struct;

// �������ƽṹ��
typedef struct
{
    quaternion_module          quaternion;               // ��Ԫ���������
    cascade_common_value_struct posture_value;          // ��̬��̬����
    pid_cycle_struct           angular_speed_cycle;      // ���ٶȻ�������
    pid_cycle_struct           angle_cycle;              // �ǶȻ�������
    pid_cycle_struct           speed_cycle;              // �ٶȻ�������
    pid_cycle_struct           track_cycle;              // ת�򻷿�����

} cascade_value_struct;


extern cascade_value_struct roll_balance_cascade;         // ����ƽ����Ʋ����ṹ��
extern cascade_value_struct roll_balance_cascade_resave;  // ����ƽ����Ʋ����ṹ���ʼ����
extern cascade_value_struct pitch_balance_cascade;        // ���ƽ����������ṹ��
extern cascade_value_struct pitch_balance_cascade_resave; // ���ƽ����������ṹ���ʼ����
extern cascade_value_struct track_cascade;   

// �������    ��Ԫ��ģ����㣨������̬���ݣ�
// ����˵��   cascade_value - �������ƽṹ��ָ�루�洢��Ԫ������̬���ݣ�
// ���ز���   void
// ʹ��ʾ��   quaternion_module_calculate(&balance_cascade);
// ��ע��Ϣ   �ú���ͨ���ں������Ǻͼ��ٶȼ����ݣ�������Ԫ������ת�������ռ������̬�ǣ�����ǡ������ǡ�ƫ���ǣ�
void quaternion_module_calculate(cascade_value_struct *cascade_value);

// �������    λ��ʽ PID ���Ƽ���
// ����˵��   pid_cycle - PID �������ṹ��ָ�룻target - Ŀ��ֵ��real - ʵ�ʲ���ֵ
// ���ز���   void
// ʹ��ʾ��   pid_control(&balance_cascade.speed_cycle, 0, (left_enc + right_enc) / 2);
// ��ע��Ϣ   ����λ��ʽ PID ��������������������֡�΢�ֻ��ڣ����Ի��ֺ���������޷�
void pid_control (pid_cycle_struct *pid_cycle, float target, float real);

// �������    ����ʽ PID ���Ƽ���
// ����˵��   pid_cycle - PID �������ṹ��ָ�룻target - Ŀ��ֵ��real - ʵ�ʲ���ֵ
// ���ز���   void
// ʹ��ʾ��   pid_control_incremental(&balance_cascade.angle_cycle, 0, balance_cascade.posture_value.pit);
// ��ע��Ϣ   ��������ʽ PID �������ͨ��ƫ��ı仯������������������ۼӺ���Ϊ������޷�
void pid_control_incremental (pid_cycle_struct *pid_cycle, float target, float real);

// �������    ��Ԫ��ģ���ʼ��
// ����˵��   cascade_value - �������ƽṹ��ָ�루��Ҫ��ʼ������Ԫ��ģ�飩
// ���ز���   void
// ʹ��ʾ��   quaternion_module_init(&balance_cascade);
// ��ע��Ϣ   ��ʼ����Ԫ��Ϊ��λ��Ԫ������̬��Ϊ 0�����ٶ��˲���ʼֵ��Ϊ��ǰ���ٶȼ����ݣ�У׼�������
void quaternion_module_init (cascade_value_struct *cascade_value);

// �������    ƽ�⼶�����Ƴ�ʼ��
// ���ز���   void
// ʹ��ʾ��   balance_cascade_init();
// ��ע��Ϣ   ��ʼ��ƽ����ƺ�ת��ƽ����Ƶļ����ṹ�������������̬У׼ϵ����PID �����ȣ��������ʼ״̬
void balance_cascade_init (void);


#endif
