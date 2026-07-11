# C++核心编程

本阶段主要针对C++==面向对象==编程技术做详细讲解，探讨C++中的核心和精髓。



## 1 内存分区模型

C++程序在执行时，将内存大方向划分为**4个区域**

- 代码区：存放函数体的二进制代码，由操作系统进行管理的
- 全局区：存放全局变量和静态变量以及常量
- 栈区：由编译器自动分配释放, 存放函数的参数值,局部变量等
- 堆区：由程序员分配和释放,若程序员不释放,程序结束时由操作系统回收







**内存四区意义：**

不同区域存放的数据，赋予不同的生命周期, 给我们更大的灵活编程





### 1.1 程序运行前

​	在程序编译后，生成了exe可执行程序，**未执行该程序前**分为两个区域

​	**代码区：**

​		存放 CPU 执行的机器指令

​		代码区是**共享**的，共享的目的是对于频繁被执行的程序，只需要在内存中有一份代码即可

​		代码区是**只读**的，使其只读的原因是防止程序意外地修改了它的指令

​	**全局区：**

​		全局变量和静态变量存放在此.

​		全局区还包含了常量区, 字符串常量和其他常量也存放在此.

​		==该区域的数据在程序结束后由操作系统释放==.













**示例：**

```c++
//全局变量
int g_a = 10;
int g_b = 10;

//全局常量
const int c_g_a = 10;
const int c_g_b = 10;

int main() {

	//局部变量
	int a = 10;
	int b = 10;

	//打印地址
	cout << "局部变量a地址为： " << (int)&a << endl;
	cout << "局部变量b地址为： " << (int)&b << endl;

	cout << "全局变量g_a地址为： " <<  (int)&g_a << endl;
	cout << "全局变量g_b地址为： " <<  (int)&g_b << endl;

	//静态变量
	static int s_a = 10;
	static int s_b = 10;

	cout << "静态变量s_a地址为： " << (int)&s_a << endl;
	cout << "静态变量s_b地址为： " << (int)&s_b << endl;

	cout << "字符串常量地址为： " << (int)&"hello world" << endl;
	cout << "字符串常量地址为： " << (int)&"hello world1" << endl;

	cout << "全局常量c_g_a地址为： " << (int)&c_g_a << endl;
	cout << "全局常量c_g_b地址为： " << (int)&c_g_b << endl;

	const int c_l_a = 10;
	const int c_l_b = 10;
	cout << "局部常量c_l_a地址为： " << (int)&c_l_a << endl;
	cout << "局部常量c_l_b地址为： " << (int)&c_l_b << endl;

	system("pause");

	return 0;
}
```

打印结果：

![1545017602518](assets/1545017602518.png)



总结：

* C++中在程序运行前分为全局区和代码区
* 代码区特点是共享和只读
* 全局区中存放全局变量、静态变量、常量
* 常量区中存放 const修饰的全局常量  和 字符串常量






### 1.2 程序运行后



​	**栈区：**

​		由编译器自动分配释放, 存放函数的参数值,局部变量等

​		注意事项：不要返回局部变量的地址，栈区开辟的数据由编译器自动释放



**示例：**

```c++
int * func()
{
	int a = 10;
	return &a;
}

int main() {

	int *p = func();

	cout << *p << endl;
	cout << *p << endl;

	system("pause");

	return 0;
}
```







​	**堆区：**

​		由程序员分配释放,若程序员不释放,程序结束时由操作系统回收

​		在C++中主要利用new在堆区开辟内存

**示例：**

```c++
int* func()
{
	int* a = new int(10);
	return a;
}

int main() {

	int *p = func();

	cout << *p << endl;
	cout << *p << endl;
    
	system("pause");

	return 0;
}
```



**总结：**

堆区数据由程序员管理开辟和释放

堆区数据利用new关键字进行开辟内存









### 1.3 new操作符



​	C++中利用==new==操作符在堆区开辟数据

​	堆区开辟的数据，由程序员手动开辟，手动释放，释放利用操作符 ==delete==

​	语法：` new 数据类型`

​	利用new创建的数据，会返回该数据对应的类型的指针



**示例1： 基本语法**

```c++
int* func()
{
	int* a = new int(10);
	return a;
}

int main() {

	int *p = func();

	cout << *p << endl;
	cout << *p << endl;

	//利用delete释放堆区数据
	delete p;

	//cout << *p << endl; //报错，释放的空间不可访问

	system("pause");

	return 0;
}
```



**示例2：开辟数组**

```c++
//堆区开辟数组
int main() {

	int* arr = new int[10];

	for (int i = 0; i < 10; i++)
	{
		arr[i] = i + 100;
	}

	for (int i = 0; i < 10; i++)
	{
		cout << arr[i] << endl;
	}
	//释放数组 delete 后加 []
	delete[] arr;

	system("pause");

	return 0;
}

```











## 2 引用

### 2.1 引用的基本使用

**作用： **给变量起别名

**语法：** `数据类型 &别名 = 原名`



**示例：**

```C++
int main() {

	int a = 10;
	int &b = a;

	cout << "a = " << a << endl;
	cout << "b = " << b << endl;

	b = 100;

	cout << "a = " << a << endl;
	cout << "b = " << b << endl;

	system("pause");

	return 0;
}
```







### 2.2 引用注意事项

* 引用必须初始化
* 引用在初始化后，不可以改变

示例：

```C++
int main() {

	int a = 10;
	int b = 20;
	//int &c; //错误，引用必须初始化
	int &c = a; //一旦初始化后，就不可以更改
	c = b; //这是赋值操作，不是更改引用

	cout << "a = " << a << endl;
	cout << "b = " << b << endl;
	cout << "c = " << c << endl;

	system("pause");

	return 0;
}
```











### 2.3 引用做函数参数

**作用：**函数传参时，可以利用引用的技术让形参修饰实参

**优点：**可以简化指针修改实参



**示例：**

```C++
//1. 值传递
void mySwap01(int a, int b) {
	int temp = a;
	a = b;
	b = temp;
}

//2. 地址传递
void mySwap02(int* a, int* b) {
	int temp = *a;
	*a = *b;
	*b = temp;
}

//3. 引用传递
void mySwap03(int& a, int& b) {
	int temp = a;
	a = b;
	b = temp;
}

int main() {

	int a = 10;
	int b = 20;

	mySwap01(a, b);
	cout << "a:" << a << " b:" << b << endl;

	mySwap02(&a, &b);
	cout << "a:" << a << " b:" << b << endl;

	mySwap03(a, b);
	cout << "a:" << a << " b:" << b << endl;

	system("pause");

	return 0;
}

```



> 总结：通过引用参数产生的效果同按地址传递是一样的。引用的语法更清楚简单













### 2.4 引用做函数返回值



作用：引用是可以作为函数的返回值存在的



注意：**不要返回局部变量引用**

用法：函数调用作为左值



**示例：**

```C++
//返回局部变量引用
int& test01() {
	int a = 10; //局部变量
	return a;
}

//返回静态变量引用
int& test02() {
	static int a = 20;
	return a;
}

int main() {

	//不能返回局部变量的引用
	int& ref = test01();
	cout << "ref = " << ref << endl;
	cout << "ref = " << ref << endl;

	//如果函数做左值，那么必须返回引用
	int& ref2 = test02();
	cout << "ref2 = " << ref2 << endl;
	cout << "ref2 = " << ref2 << endl;

	test02() = 1000;

	cout << "ref2 = " << ref2 << endl;
	cout << "ref2 = " << ref2 << endl;

	system("pause");

	return 0;
}
```





​	









### 2.5 引用的本质

本质：**引用的本质在c++内部实现是一个指针常量.**

讲解示例：

```C++
//发现是引用，转换为 int* const ref = &a;
void func(int& ref){
	ref = 100; // ref是引用，转换为*ref = 100
}
int main(){
	int a = 10;
    
    //自动转换为 int* const ref = &a; 指针常量是指针指向不可改，也说明为什么引用不可更改
	int& ref = a; 
	ref = 20; //内部发现ref是引用，自动帮我们转换为: *ref = 20;
    
	cout << "a:" << a << endl;
	cout << "ref:" << ref << endl;
    
	func(a);
	return 0;
}
```

结论：C++推荐用引用技术，因为语法方便，引用本质是指针常量，但是所有的指针操作编译器都帮我们做了













### 2.6 常量引用



**作用：**常量引用主要用来修饰形参，防止误操作



在函数形参列表中，可以加==const修饰形参==，防止形参改变实参



**示例：**



```C++
//引用使用的场景，通常用来修饰形参
void showValue(const int& v) {
	//v += 10;
	cout << v << endl;
}

int main() {

	//int& ref = 10;  引用本身需要一个合法的内存空间，因此这行错误
	//加入const就可以了，编译器优化代码，int temp = 10; const int& ref = temp;
	const int& ref = 10;

	//ref = 100;  //加入const后不可以修改变量
	cout << ref << endl;

	//函数中利用常量引用防止误操作修改实参
	int a = 10;
	showValue(a);

	system("pause");

	return 0;
}
```









## 3 函数提高

### 3.1 函数默认参数



在C++中，函数的形参列表中的形参是可以有默认值的。

语法：` 返回值类型  函数名 （参数= 默认值）{}`



**示例：**

```C++
int func(int a, int b = 10, int c = 10) {
	return a + b + c;
}

//1. 如果某个位置参数有默认值，那么从这个位置往后，从左向右，必须都要有默认值
//2. 如果函数声明有默认值，函数实现的时候就不能有默认参数
int func2(int a = 10, int b = 10);
int func2(int a, int b) {
	return a + b;
}

int main() {

	cout << "ret = " << func(20, 20) << endl;
	cout << "ret = " << func(100) << endl;

	system("pause");

	return 0;
}
```







### 3.2 函数占位参数



C++中函数的形参列表里可以有占位参数，用来做占位，调用函数时必须填补该位置



**语法：** `返回值类型 函数名 (数据类型){}`



在现阶段函数的占位参数存在意义不大，但是后面的课程中会用到该技术



**示例：**

```C++
//函数占位参数 ，占位参数也可以有默认参数
void func(int a, int) {
	cout << "this is func" << endl;
}

int main() {

	func(10,10); //占位参数必须填补

	system("pause");

	return 0;
}
```









### 3.3 函数重载

#### 3.3.1 函数重载概述



**作用：**函数名可以相同，提高复用性



**函数重载满足条件：**

* 同一个作用域下
* 函数名称相同
* 函数参数**类型不同**  或者 **个数不同** 或者 **顺序不同**



**注意:**  函数的返回值不可以作为函数重载的条件



**示例：**

```C++
//函数重载需要函数都在同一个作用域下
void func()
{
	cout << "func 的调用！" << endl;
}
void func(int a)
{
	cout << "func (int a) 的调用！" << endl;
}
void func(double a)
{
	cout << "func (double a)的调用！" << endl;
}
void func(int a ,double b)
{
	cout << "func (int a ,double b) 的调用！" << endl;
}
void func(double a ,int b)
{
	cout << "func (double a ,int b)的调用！" << endl;
}

//函数返回值不可以作为函数重载条件
//int func(double a, int b)
//{
//	cout << "func (double a ,int b)的调用！" << endl;
//}


int main() {

	func();
	func(10);
	func(3.14);
	func(10,3.14);
	func(3.14 , 10);
	
	system("pause");

	return 0;
}
```













#### 3.3.2 函数重载注意事项



* 引用作为重载条件
* 函数重载碰到函数默认参数





**示例：**

```C++
//函数重载注意事项
//1、引用作为重载条件

void func(int &a)
{
	cout << "func (int &a) 调用 " << endl;
}

void func(const int &a)
{
	cout << "func (const int &a) 调用 " << endl;
}


//2、函数重载碰到函数默认参数

void func2(int a, int b = 10)
{
	cout << "func2(int a, int b = 10) 调用" << endl;
}

void func2(int a)
{
	cout << "func2(int a) 调用" << endl;
}

int main() {
	
	int a = 10;
	func(a); //调用无const
	func(10);//调用有const


	//func2(10); //碰到默认参数产生歧义，需要避免

	system("pause");

	return 0;
}
```







## **4** 类和对象



C++面向对象的三大特性为：==封装、继承、多态==



C++认为==万事万物都皆为对象==，对象上有其属性和行为



**例如：**

​	人可以作为对象，属性有姓名、年龄、身高、体重...，行为有走、跑、跳、吃饭、唱歌...

​	车也可以作为对象，属性有轮胎、方向盘、车灯...,行为有载人、放音乐、放空调...

​	具有相同性质的==对象==，我们可以抽象称为==类==，人属于人类，车属于车类

### 4.1 封装

#### 4.1.1  封装的意义

封装是C++面向对象三大特性之一

封装的意义：

* 将属性和行为作为一个整体，表现生活中的事物
* 将属性和行为加以权限控制



**封装意义一：**

​	在设计类的时候，属性和行为写在一起，表现事物

**语法：** `class 类名{   访问权限： 属性  / 行为  };`



**示例1：**设计一个圆类，求圆的周长

**示例代码：**

```C++
//圆周率
const double PI = 3.14;

//1、封装的意义
//将属性和行为作为一个整体，用来表现生活中的事物

//封装一个圆类，求圆的周长
//class代表设计一个类，后面跟着的是类名
class Circle
{
public:  //访问权限  公共的权限

	//属性
	int m_r;//半径

	//行为
	//获取到圆的周长
	double calculateZC()
	{
		//2 * pi  * r
		//获取圆的周长
		return  2 * PI * m_r;
	}
};

int main() {

	//通过圆类，创建圆的对象
	// c1就是一个具体的圆
	Circle c1;
	c1.m_r = 10; //给圆对象的半径 进行赋值操作

	//2 * pi * 10 = = 62.8
	cout << "圆的周长为： " << c1.calculateZC() << endl;

	system("pause");

	return 0;
}
```





**示例2：**设计一个学生类，属性有姓名和学号，可以给姓名和学号赋值，可以显示学生的姓名和学号





**示例2代码：**

```C++
//学生类
class Student {
public:
	void setName(string name) {
		m_name = name;
	}
	void setID(int id) {
		m_id = id;
	}

	void showStudent() {
		cout << "name:" << m_name << " ID:" << m_id << endl;
	}
public:
	string m_name;
	int m_id;
};

int main() {

	Student stu;
	stu.setName("德玛西亚");
	stu.setID(250);
	stu.showStudent();

	system("pause");

	return 0;
}

```









**封装意义二：**

类在设计时，可以把属性和行为放在不同的权限下，加以控制

访问权限有三种：



1. public        公共权限  
2. protected 保护权限
3. private      私有权限







**示例：**

```C++
//三种权限
//公共权限  public     类内可以访问  类外可以访问
//保护权限  protected  类内可以访问  类外不可以访问
//私有权限  private    类内可以访问  类外不可以访问

class Person
{
	//姓名  公共权限
public:
	string m_Name;

	//汽车  保护权限
protected:
	string m_Car;

	//银行卡密码  私有权限
private:
	int m_Password;

public:
	void func()
	{
		m_Name = "张三";
		m_Car = "拖拉机";
		m_Password = 123456;
	}
};

int main() {

	Person p;
	p.m_Name = "李四";
	//p.m_Car = "奔驰";  //保护权限类外访问不到
	//p.m_Password = 123; //私有权限类外访问不到

	system("pause");

	return 0;
}
```







#### 4.1.2 struct和class区别



在C++中 struct和class唯一的**区别**就在于 **默认的访问权限不同**

区别：

* struct 默认权限为公共
* class   默认权限为私有



```C++
class C1
{
	int  m_A; //默认是私有权限
};

struct C2
{
	int m_A;  //默认是公共权限
};

int main() {

	C1 c1;
	c1.m_A = 10; //错误，访问权限是私有

	C2 c2;
	c2.m_A = 10; //正确，访问权限是公共

	system("pause");

	return 0;
}
```













#### 4.1.3 成员属性设置为私有



**优点1：**将所有成员属性设置为私有，可以自己控制读写权限

**优点2：**对于写权限，我们可以检测数据的有效性



**示例：**

```C++
class Person {
public:

	//姓名设置可读可写
	void setName(string name) {
		m_Name = name;
	}
	string getName()
	{
		return m_Name;
	}


	//获取年龄 
	int getAge() {
		return m_Age;
	}
	//设置年龄
	void setAge(int age) {
		if (age < 0 || age > 150) {
			cout << "你个老妖精!" << endl;
			return;
		}
		m_Age = age;
	}

	//情人设置为只写
	void setLover(string lover) {
		m_Lover = lover;
	}

private:
	string m_Name; //可读可写  姓名
	
	int m_Age; //只读  年龄

	string m_Lover; //只写  情人
};


int main() {

	Person p;
	//姓名设置
	p.setName("张三");
	cout << "姓名： " << p.getName() << endl;

	//年龄设置
	p.setAge(50);
	cout << "年龄： " << p.getAge() << endl;

	//情人设置
	p.setLover("苍井");
	//cout << "情人： " << p.m_Lover << endl;  //只写属性，不可以读取

	system("pause");

	return 0;
}
```









**练习案例1：设计立方体类**

设计立方体类(Cube)

求出立方体的面积和体积

分别用全局函数和成员函数判断两个立方体是否相等。



![1545533548532](assets/1545533548532.png)











**练习案例2：点和圆的关系**

设计一个圆形类（Circle），和一个点类（Point），计算点和圆的关系。



![1545533829184](assets/1545533829184.png)







### 4.2 对象的初始化和清理



*  生活中我们买的电子产品都基本会有出厂设置，在某一天我们不用时候也会删除一些自己信息数据保证安全
*  C++中的面向对象来源于生活，每个对象也都会有初始设置以及 对象销毁前的清理数据的设置。





#### 4.2.1 构造函数和析构函数

对象的**初始化和清理**也是两个非常重要的安全问题

​	一个对象或者变量没有初始状态，对其使用后果是未知

​	同样的使用完一个对象或变量，没有及时清理，也会造成一定的安全问题



c++利用了**构造函数**和**析构函数**解决上述问题，这两个函数将会被编译器自动调用，完成对象初始化和清理工作。

对象的初始化和清理工作是编译器强制要我们做的事情，因此如果**我们不提供构造和析构，编译器会提供**

**编译器提供的构造函数和析构函数是空实现。**



* 构造函数：主要作用在于创建对象时为对象的成员属性赋值，构造函数由编译器自动调用，无须手动调用。
* 析构函数：主要作用在于对象**销毁前**系统自动调用，执行一些清理工作。





**构造函数语法：**`类名(){}`

1. 构造函数，没有返回值也不写void
2. 函数名称与类名相同
3. 构造函数可以有参数，因此可以发生重载
4. 程序在调用对象时候会自动调用构造，无须手动调用,而且只会调用一次





**析构函数语法：** `~类名(){}`

1. 析构函数，没有返回值也不写void
2. 函数名称与类名相同,在名称前加上符号  ~
3. 析构函数不可以有参数，因此不可以发生重载
4. 程序在对象销毁前会自动调用析构，无须手动调用,而且只会调用一次





```C++
class Person
{
public:
	//构造函数
	Person()
	{
		cout << "Person的构造函数调用" << endl;
	}
	//析构函数
	~Person()
	{
		cout << "Person的析构函数调用" << endl;
	}

};

void test01()
{
	Person p;
}

int main() {
	
	test01();

	system("pause");

	return 0;
}
```











#### 4.2.2 构造函数的分类及调用

两种分类方式：

​	按参数分为： 有参构造和无参构造

​	按类型分为： 普通构造和拷贝构造

三种调用方式：

​	括号法

​	显示法

​	隐式转换法



**示例：**

```C++
//1、构造函数分类
// 按照参数分类分为 有参和无参构造   无参又称为默认构造函数
// 按照类型分类分为 普通构造和拷贝构造

class Person {
public:
	//无参（默认）构造函数
	Person() {
		cout << "无参构造函数!" << endl;
	}
	//有参构造函数
	Person(int a) {
		age = a;
		cout << "有参构造函数!" << endl;
	}
	//拷贝构造函数
	Person(const Person& p) {
		age = p.age;
		cout << "拷贝构造函数!" << endl;
	}
	//析构函数
	~Person() {
		cout << "析构函数!" << endl;
	}
public:
	int age;
};

//2、构造函数的调用
//调用无参构造函数
void test01() {
	Person p; //调用无参构造函数
}
//调用有参的构造函数
void test02() {

	//2.1  括号法，常用
	Person p1(10);
	//注意1：调用无参构造函数不能加括号，如果加了编译器认为这是一个函数声明
	//Person p2();

	//2.2 显式法
	Person p2 = Person(10); 
	Person p3 = Person(p2);
	//Person(10)单独写就是匿名对象  当前行结束之后，马上析构

	//2.3 隐式转换法
	Person p4 = 10; // Person p4 = Person(10); 
	Person p5 = p4; // Person p5 = Person(p4); 

	//注意2：不能利用 拷贝构造函数 初始化匿名对象 编译器认为是对象声明
	//Person p5(p4);
}

int main() {

	test01();
	//test02();

	system("pause");

	return 0;
}
```









#### 4.2.3 拷贝构造函数调用时机



C++中拷贝构造函数调用时机通常有三种情况

* 使用一个已经创建完毕的对象来初始化一个新对象
* 值传递的方式给函数参数传值
* 以值方式返回局部对象



**示例：**

```C++
class Person {
public:
	Person() {
		cout << "无参构造函数!" << endl;
		mAge = 0;
	}
	Person(int age) {
		cout << "有参构造函数!" << endl;
		mAge = age;
	}
	Person(const Person& p) {
		cout << "拷贝构造函数!" << endl;
		mAge = p.mAge;
	}
	//析构函数在释放内存之前调用
	~Person() {
		cout << "析构函数!" << endl;
	}
public:
	int mAge;
};

//1. 使用一个已经创建完毕的对象来初始化一个新对象
void test01() {

	Person man(100); //p对象已经创建完毕
	Person newman(man); //调用拷贝构造函数
	Person newman2 = man; //拷贝构造

	//Person newman3;
	//newman3 = man; //不是调用拷贝构造函数，赋值操作
}

//2. 值传递的方式给函数参数传值
//相当于Person p1 = p;
void doWork(Person p1) {}
void test02() {
	Person p; //无参构造函数
	doWork(p);
}

//3. 以值方式返回局部对象
Person doWork2()
{
	Person p1;
	cout << (int *)&p1 << endl;
	return p1;
}

void test03()
{
	Person p = doWork2();
	cout << (int *)&p << endl;
}


int main() {

	//test01();
	//test02();
	test03();

	system("pause");

	return 0;
}
```





#### 4.2.4 构造函数调用规则

默认情况下，c++编译器至少给一个类添加3个函数

1．默认构造函数(无参，函数体为空)

2．默认析构函数(无参，函数体为空)

3．默认拷贝构造函数，对属性进行值拷贝



构造函数调用规则如下：

* 如果用户定义有参构造函数，c++不在提供默认无参构造，但是会提供默认拷贝构造


* 如果用户定义拷贝构造函数，c++不会再提供其他构造函数



示例：

```C++
class Person {
public:
	//无参（默认）构造函数
	Person() {
		cout << "无参构造函数!" << endl;
	}
	//有参构造函数
	Person(int a) {
		age = a;
		cout << "有参构造函数!" << endl;
	}
	//拷贝构造函数
	Person(const Person& p) {
		age = p.age;
		cout << "拷贝构造函数!" << endl;
	}
	//析构函数
	~Person() {
		cout << "析构函数!" << endl;
	}
public:
	int age;
};

void test01()
{
	Person p1(18);
	//如果不写拷贝构造，编译器会自动添加拷贝构造，并且做浅拷贝操作
	Person p2(p1);

	cout << "p2的年龄为： " << p2.age << endl;
}

void test02()
{
	//如果用户提供有参构造，编译器不会提供默认构造，会提供拷贝构造
	Person p1; //此时如果用户自己没有提供默认构造，会出错
	Person p2(10); //用户提供的有参
	Person p3(p2); //此时如果用户没有提供拷贝构造，编译器会提供

	//如果用户提供拷贝构造，编译器不会提供其他构造函数
	Person p4; //此时如果用户自己没有提供默认构造，会出错
	Person p5(10); //此时如果用户自己没有提供有参，会出错
	Person p6(p5); //用户自己提供拷贝构造
}

int main() {

	test01();

	system("pause");

	return 0;
}
```









#### 4.2.5 深拷贝与浅拷贝



深浅拷贝是面试经典问题，也是常见的一个坑



浅拷贝：简单的赋值拷贝操作



深拷贝：在堆区重新申请空间，进行拷贝操作



**示例：**

```C++
class Person {
public:
	//无参（默认）构造函数
	Person() {
		cout << "无参构造函数!" << endl;
	}
	//有参构造函数
	Person(int age ,int height) {
		
		cout << "有参构造函数!" << endl;

		m_age = age;
		m_height = new int(height);
		
	}
	//拷贝构造函数  
	Person(const Person& p) {
		cout << "拷贝构造函数!" << endl;
		//如果不利用深拷贝在堆区创建新内存，会导致浅拷贝带来的重复释放堆区问题
		m_age = p.m_age;
		m_height = new int(*p.m_height);
		
	}

	//析构函数
	~Person() {
		cout << "析构函数!" << endl;
		if (m_height != NULL)
		{
			delete m_height;
		}
	}
public:
	int m_age;
	int* m_height;
};

void test01()
{
	Person p1(18, 180);

	Person p2(p1);

	cout << "p1的年龄： " << p1.m_age << " 身高： " << *p1.m_height << endl;

	cout << "p2的年龄： " << p2.m_age << " 身高： " << *p2.m_height << endl;
}

int main() {

	test01();

	system("pause");

	return 0;
}
```

> 总结：如果属性有在堆区开辟的，一定要自己提供拷贝构造函数，防止浅拷贝带来的问题









#### 4.2.6 初始化列表



**作用：**

C++提供了初始化列表语法，用来初始化属性



**语法：**`构造函数()：属性1(值1),属性2（值2）... {}`



**示例：**

```C++
class Person {
public:

	////传统方式初始化
	//Person(int a, int b, int c) {
	//	m_A = a;
	//	m_B = b;
	//	m_C = c;
	//}

	//初始化列表方式初始化
	Person(int a, int b, int c) :m_A(a), m_B(b), m_C(c) {}
	void PrintPerson() {
		cout << "mA:" << m_A << endl;
		cout << "mB:" << m_B << endl;
		cout << "mC:" << m_C << endl;
	}
private:
	int m_A;
	int m_B;
	int m_C;
};

int main() {

	Person p(1, 2, 3);
	p.PrintPerson();


	system("pause");

	return 0;
}
```





#### 4.2.7 类对象作为类成员



C++类中的成员可以是另一个类的对象，我们称该成员为 对象成员



例如：

```C++
class A {}
class B
{
    A a；
}
```



B类中有对象A作为成员，A为对象成员



那么当创建B对象时，A与B的构造和析构的顺序是谁先谁后？
* 先构造类对象
* 再构造自身
* 析构的顺序与构造相反



**示例：**

```C++
class Phone
{
public:
	Phone(string name)
	{
		m_PhoneName = name;
		cout << "Phone构造" << endl;
	}

	~Phone()
	{
		cout << "Phone析构" << endl;
	}

	string m_PhoneName;

};


class Person
{
public:

	//初始化列表可以告诉编译器调用哪一个构造函数
	Person(string name, string pName) :m_Name(name), m_Phone(pName)
	{
		cout << "Person构造" << endl;
	}

	~Person()
	{
		cout << "Person析构" << endl;
	}

	void playGame()
	{
		cout << m_Name << " 使用" << m_Phone.m_PhoneName << " 牌手机! " << endl;
	}

	string m_Name;
	Phone m_Phone;

};
void test01()
{
	//当类中成员是其他类对象时，我们称该成员为 对象成员
	//构造的顺序是 ：先调用对象成员的构造，再调用本类构造
	//析构顺序与构造相反
	Person p("张三" , "苹果X");
	p.playGame();

}


int main() {

	test01();

	system("pause");

	return 0;
}
```











#### 4.2.8 静态成员

静态成员就是在成员变量和成员函数前加上关键字static，称为静态成员

静态成员分为：



*  静态成员变量
   *  所有对象共享同一份数据
   *  在编译阶段分配内存
   *  类内声明，类外初始化
*  静态成员函数
   *  所有对象共享同一个函数
   *  静态成员函数只能访问静态成员变量







**示例1 ：**静态成员变量

```C++
class Person
{
	
public:

	static int m_A; //静态成员变量

	//静态成员变量特点：
	//1 在编译阶段分配内存
	//2 类内声明，类外初始化
	//3 所有对象共享同一份数据

private:
	static int m_B; //静态成员变量也是有访问权限的
};
int Person::m_A = 10;
int Person::m_B = 10;

void test01()
{
	//静态成员变量两种访问方式

	//1、通过对象
	Person p1;
	p1.m_A = 100;
	cout << "p1.m_A = " << p1.m_A << endl;
	//打印为100，

	Person p2;
	p2.m_A = 200;
	cout << "p1.m_A = " << p1.m_A << endl; //共享同一份数据
	cout << "p2.m_A = " << p2.m_A << endl;
	//打印为200，因为m_A是共享的，所以p1和p2的m_A都是200

	//2、通过类名
	cout << "m_A = " << Person::m_A << endl;


	//cout << "m_B = " << Person::m_B << endl; //私有权限访问不到
}

int main() {

	test01();

	system("pause");

	return 0;
}
```



**示例2：**静态成员函数

```C++
class Person
{

public:

	//静态成员函数特点：
	//1 程序共享一个函数
	//2 静态成员函数只能访问静态成员变量
	
	static void func()
	{
		cout << "func调用" << endl;
		m_A = 100;
		//m_B = 100; //错误，不可以访问非静态成员变量
	}

	static int m_A; //静态成员变量
	int m_B; // 
private:

	//静态成员函数也是有访问权限的
	static void func2()
	{
		cout << "func2调用" << endl;
	}
};
int Person::m_A = 10;


void test01()
{
	//静态成员变量两种访问方式

	//1、通过对象
	Person p1;
	p1.func();

	//2、通过类名
	Person::func();


	//Person::func2(); //私有权限访问不到
}

int main() {

	test01();

	system("pause");

	return 0;
}
```









### 4.3 C++对象模型和this指针



#### 4.3.1 成员变量和成员函数分开存储



在C++中，类内的成员变量和成员函数分开存储

只有非静态成员变量才属于类的对象上



```C++
class Person {
public:
	Person() {
		mA = 0;
	}
	//非静态成员变量占对象空间
	int mA;
	//静态成员变量不占对象空间
	static int mB; 
	//函数也不占对象空间，所有函数共享一个函数实例
	void func() {
		cout << "mA:" << this->mA << endl;
	}
	//静态成员函数也不占对象空间
	static void sfunc() {
	}
};

int main() {

	cout << sizeof(Person) << endl;

	system("pause");

	return 0;
}
```







#### 4.3.2 this指针概念

通过4.3.1我们知道在C++中成员变量和成员函数是分开存储的

每一个非静态成员函数只会诞生一份函数实例，也就是说多个同类型的对象会共用一块代码

那么问题是：这一块代码是如何区分那个对象调用自己的呢？



c++通过提供特殊的对象指针，this指针，解决上述问题。**this指针指向被调用的成员函数所属的对象**



this指针是隐含每一个非静态成员函数内的一种指针

this指针不需要定义，直接使用即可



this指针的用途：

*  当形参和成员变量同名时，可用this指针来区分
*  在类的非静态成员函数中返回对象本身，可使用return *this

```C++
class Person
{
public:

	Person(int age)
	{
		//1、当形参和成员变量同名时，可用this指针来区分
		this->age = age;
	}

	Person& PersonAddPerson(Person p)
	{
		this->age += p.age;
		//返回对象本身
		return *this;
	}

	int age;
};

void test01()
{
	Person p1(10);
	cout << "p1.age = " << p1.age << endl;

	Person p2(10);
	p2.PersonAddPerson(p1).PersonAddPerson(p1).PersonAddPerson(p1);
	cout << "p2.age = " << p2.age << endl;
}

int main() {

	test01();

	system("pause");

	return 0;
}
```









#### 4.3.3 空指针访问成员函数



C++中空指针也是可以调用成员函数的，但是也要注意有没有用到this指针



如果用到this指针，需要加以判断保证代码的健壮性



**示例：**

```C++
//空指针访问成员函数
class Person {
public:

	void ShowClassName() {
		cout << "我是Person类!" << endl;
	}

	void ShowPerson() {
		if (this == NULL) {
			return;
		}
		cout << mAge << endl;
	}

public:
	int mAge;
};

void test01()
{
	Person * p = NULL;
	p->ShowClassName(); //空指针，可以调用成员函数
	p->ShowPerson();  //但是如果成员函数中用到了this指针，就不可以了
}
 //但是如果成员函数中用到了this指针，就不可以了，因为this指针指向的是对象的地址，而空指针是没有地址的，所以会报错
 //解决方法是在成员函数中加上判断，如果this指针为空，就返回，否则就继续执行。

int main() {

	test01();

	system("pause");

	return 0;
}
```









#### 4.3.4 const修饰成员函数



**常函数：**

* 成员函数后加const后我们称为这个函数为**常函数**
* 常函数内不可以修改成员属性。
* 成员属性声明时加关键字mutable后，在常函数中依然可以修改



**常对象：**

* 声明对象前加const称该对象为常对象
* 常对象只能调用常函数







**示例：**

```C++
class Person {
public:
	Person() {
		m_A = 0;
		m_B = 0;
	}

	//this指针的本质是一个指针常量，指针的指向不可修改
	//如果想让指针指向的值也不可以修改，需要声明常函数
	void ShowPerson() const {
		//const Type* const pointer;
		//this = NULL; //不能修改指针的指向 Person* const this;
		//this->mA = 100; //但是this指针指向的对象的数据是可以修改的

		//const修饰成员函数，表示指针指向的内存空间的数据不能修改，除了mutable修饰的变量
		this->m_B = 100;
	}

	void MyFunc() const {
		//mA = 10000;
	}

public:
	int m_A;
	mutable int m_B; //可修改 可变的
};


//const修饰对象  常对象
void test01() {

	const Person person; //常量对象  
	cout << person.m_A << endl;
	//person.mA = 100; //常对象不能修改成员变量的值,但是可以访问
	person.m_B = 100; //但是常对象可以修改mutable修饰成员变量

	//常对象访问成员函数
	person.MyFunc(); //常对象可以调用 const 成员函数。

}

int main() {

	test01();

	system("pause");

	return 0;
}
```








### 4.4 友元



生活中你的家有客厅(Public)，有你的卧室(Private)

客厅所有来的客人都可以进去，但是你的卧室是私有的，也就是说只有你能进去

但是呢，你也可以允许你的好闺蜜好基友进去。



在程序里，有些私有属性 也想让类外特殊的一些函数或者类进行访问，就需要用到友元的技术



友元的目的就是让一个函数或者类 访问另一个类中私有成员



友元的关键字为  ==friend==



友元的三种实现

* 全局函数做友元
* 类做友元
* 成员函数做友元





#### 4.4.1 全局函数做友元

```C++
class Building
{
	//告诉编译器 goodGay全局函数 是 Building类的好朋友，可以访问类中的私有内容
	friend void goodGay(Building * building);

public:

	Building()
	{
		this->m_SittingRoom = "客厅";
		this->m_BedRoom = "卧室";
	}


public:
	string m_SittingRoom; //客厅

private:
	string m_BedRoom; //卧室
};


void goodGay(Building * building)
{
	cout << "好基友正在访问： " << building->m_SittingRoom << endl;
	cout << "好基友正在访问： " << building->m_BedRoom << endl;
}


void test01()
{
	Building b;
	goodGay(&b);
}

int main(){

	test01();

	system("pause");
	return 0;
}
```



#### 4.4.2 类做友元



```C++
class Building;
class goodGay
{
public:

	goodGay();
	void visit();

private:
	Building *building;
};


class Building
{
	//告诉编译器 goodGay类是Building类的好朋友，可以访问到Building类中私有内容
	friend class goodGay;

public:
	Building();

public:
	string m_SittingRoom; //客厅
private:
	string m_BedRoom;//卧室
};

Building::Building()
{
	this->m_SittingRoom = "客厅";
	this->m_BedRoom = "卧室";
}

goodGay::goodGay()
{
	building = new Building;
}

void goodGay::visit()
{
	cout << "好基友正在访问" << building->m_SittingRoom << endl;
	cout << "好基友正在访问" << building->m_BedRoom << endl;
}

void test01()
{
	goodGay gg;
	gg.visit();

}

int main(){

	test01();

	system("pause");
	return 0;
}
```





#### 4.4.3 成员函数做友元



```C++

class Building;
class goodGay
{
public:

	goodGay();
	void visit(); //只让visit函数作为Building的好朋友，可以发访问Building中私有内容
	void visit2(); 

private:
	Building *building;
};


class Building
{
	//告诉编译器  goodGay类中的visit成员函数 是Building好朋友，可以访问私有内容
	friend void goodGay::visit();

public:
	Building();

public:
	string m_SittingRoom; //客厅
private:
	string m_BedRoom;//卧室
};

Building::Building()
{
	this->m_SittingRoom = "客厅";
	this->m_BedRoom = "卧室";
}

goodGay::goodGay()
{
	building = new Building;
}

void goodGay::visit()
{
	cout << "好基友正在访问" << building->m_SittingRoom << endl;
	cout << "好基友正在访问" << building->m_BedRoom << endl;
}

void goodGay::visit2()
{
	cout << "好基友正在访问" << building->m_SittingRoom << endl;
	//cout << "好基友正在访问" << building->m_BedRoom << endl;
}

void test01()
{
	goodGay  gg;
	gg.visit();

}

int main(){
    
	test01();

	system("pause");
	return 0;
}
```









### 4.5 运算符重载



运算符重载概念：对已有的运算符重新进行定义，赋予其另一种功能，以适应不同的数据类型



#### 4.5.1 加号运算符重载



作用：实现两个自定义数据类型相加的运算



```C++
class Person {
public:
	Person() {};
	Person(int a, int b)
	{
		this->m_A = a;
		this->m_B = b;
	}
	//成员函数实现 + 号运算符重载
	Person operator+(const Person& p) {
		Person temp;
		temp.m_A = this->m_A + p.m_A;
		temp.m_B = this->m_B + p.m_B;
		return temp;
	}


public:
	int m_A;
	int m_B;
};

//全局函数实现 + 号运算符重载
//Person operator+(const Person& p1, const Person& p2) {
//	Person temp(0, 0);
//	temp.m_A = p1.m_A + p2.m_A;
//	temp.m_B = p1.m_B + p2.m_B;
//	return temp;
//}

//运算符重载 可以发生函数重载 
Person operator+(const Person& p2, int val)  
{
	Person temp;
	temp.m_A = p2.m_A + val;
	temp.m_B = p2.m_B + val;
	return temp;
}

void test() {

	Person p1(10, 10);
	Person p2(20, 20);

	//成员函数方式
	Person p3 = p2 + p1;  //相当于 p2.operaor+(p1)
	cout << "mA:" << p3.m_A << " mB:" << p3.m_B << endl;


	Person p4 = p3 + 10; //相当于 operator+(p3,10)
	cout << "mA:" << p4.m_A << " mB:" << p4.m_B << endl;

}

int main() {

	test();

	system("pause");

	return 0;
}
```



> 总结1：对于内置的数据类型的表达式的的运算符是不可能改变的

> 总结2：不要滥用运算符重载







#### 4.5.2 左移运算符重载



作用：可以输出自定义数据类型



```C++
class Person {
	friend ostream& operator<<(ostream& out, Person& p);

public:

	Person(int a, int b)
	{
		this->m_A = a;
		this->m_B = b;
	}

	//成员函数 实现不了  p << cout 不是我们想要的效果
	//void operator<<(Person& p){
	//}

private:
	int m_A;
	int m_B;
};

//全局函数实现左移重载
//ostream对象只能有一个
ostream& operator<<(ostream& out, Person& p) {
	out << "a:" << p.m_A << " b:" << p.m_B;
	return out;
}

void test() {

	Person p1(10, 20);

	cout << p1 << "hello world" << endl; //链式编程
}

int main() {

	test();

	system("pause");

	return 0;
}
```



> 总结：重载左移运算符配合友元可以实现输出自定义数据类型













#### 4.5.3 递增运算符重载



作用： 通过重载递增运算符，实现自己的整型数据



```C++

class MyInteger {

	friend ostream& operator<<(ostream& out, MyInteger myint);

public:
	MyInteger() {
		m_Num = 0;
	}
	//前置++
	MyInteger& operator++() {
		//先++
		m_Num++;
		//再返回
		return *this;
	}

	//后置++
	MyInteger operator++(int) {
		//先返回
		MyInteger temp = *this; //记录当前本身的值，然后让本身的值加1，但是返回的是以前的值，达到先返回后++；
		m_Num++;
		return temp;
	}

private:
	int m_Num;
};


ostream& operator<<(ostream& out, MyInteger myint) {
	out << myint.m_Num;
	return out;
}


//前置++ 先++ 再返回
void test01() {
	MyInteger myInt;
	cout << ++myInt << endl;
	cout << myInt << endl;
}

//后置++ 先返回 再++
void test02() {

	MyInteger myInt;
	cout << myInt++ << endl;
	cout << myInt << endl;
}

int main() {

	test01();
	//test02();

	system("pause");

	return 0;
}
```



> 总结： 前置递增返回引用，后置递增返回值













#### 4.5.4 赋值运算符重载



c++编译器至少给一个类添加4个函数

1. 默认构造函数(无参，函数体为空)
2. 默认析构函数(无参，函数体为空)
3. 默认拷贝构造函数，对属性进行值拷贝
4. 赋值运算符 operator=, 对属性进行值拷贝





如果类中有属性指向堆区，做赋值操作时也会出现深浅拷贝问题





**示例：**

```C++
class Person
{
public:

	Person(int age)
	{
		//将年龄数据开辟到堆区
		m_Age = new int(age);
	}

	//重载赋值运算符 
	Person& operator=(Person &p)
	{
		if (m_Age != NULL)
		{
			delete m_Age;
			m_Age = NULL;
		}
		//编译器提供的代码是浅拷贝
		//m_Age = p.m_Age;

		//提供深拷贝 解决浅拷贝的问题
		m_Age = new int(*p.m_Age);

		//返回自身
		return *this;
	}


	~Person()
	{
		if (m_Age != NULL)
		{
			delete m_Age;
			m_Age = NULL;
		}
	}

	//年龄的指针
	int *m_Age;

};


void test01()
{
	Person p1(18);

	Person p2(20);

	Person p3(30);

	p3 = p2 = p1; //赋值操作

	cout << "p1的年龄为：" << *p1.m_Age << endl;

	cout << "p2的年龄为：" << *p2.m_Age << endl;

	cout << "p3的年龄为：" << *p3.m_Age << endl;
}

int main() {

	test01();

	//int a = 10;
	//int b = 20;
	//int c = 30;

	//c = b = a;
	//cout << "a = " << a << endl;
	//cout << "b = " << b << endl;
	//cout << "c = " << c << endl;

	system("pause");

	return 0;
}
```









#### 4.5.5 关系运算符重载



**作用：**重载关系运算符，可以让两个自定义类型对象进行对比操作



**示例：**

```C++
class Person
{
public:
	Person(string name, int age)
	{
		this->m_Name = name;
		this->m_Age = age;
	};

	bool operator==(Person & p)
	{
		if (this->m_Name == p.m_Name && this->m_Age == p.m_Age)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	bool operator!=(Person & p)
	{
		if (this->m_Name == p.m_Name && this->m_Age == p.m_Age)
		{
			return false;
		}
		else
		{
			return true;
		}
	}

	string m_Name;
	int m_Age;
};

void test01()
{
	//int a = 0;
	//int b = 0;

	Person a("孙悟空", 18);
	Person b("孙悟空", 18);

	if (a == b)
	{
		cout << "a和b相等" << endl;
	}
	else
	{
		cout << "a和b不相等" << endl;
	}

	if (a != b)
	{
		cout << "a和b不相等" << endl;
	}
	else
	{
		cout << "a和b相等" << endl;
	}
}


int main() {

	test01();

	system("pause");

	return 0;
}
```





#### 4.5.6 函数调用运算符重载



* 函数调用运算符 ()  也可以重载
* 由于重载后使用的方式非常像函数的调用，因此称为仿函数
* 仿函数没有固定写法，非常灵活



**示例：**

```C++
class MyPrint
{
public:
	void operator()(string text)
	{
		cout << text << endl;
	}

};
void test01()
{
	//重载的（）操作符 也称为仿函数
	MyPrint myFunc;
	myFunc("hello world");
}


class MyAdd
{
public:
	int operator()(int v1, int v2)
	{
		return v1 + v2;
	}
};

void test02()
{
	MyAdd add;
	int ret = add(10, 10);
	cout << "ret = " << ret << endl;

	//匿名对象调用  
	cout << "MyAdd()(100,100) = " << MyAdd()(100, 100) << endl;
}

int main() {

	test01();
	test02();

	system("pause");

	return 0;
}
```









### 4.6  继承

**继承是面向对象三大特性之一**

有些类与类之间存在特殊的关系，例如下图中：

![1544861202252](assets/1544861202252.png)

我们发现，定义这些类时，下级别的成员除了拥有上一级的共性，还有自己的特性。

这个时候我们就可以考虑利用继承的技术，减少重复代码



#### 4.6.1 继承的基本语法



例如我们看到很多网站中，都有公共的头部，公共的底部，甚至公共的左侧列表，只有中心内容不同

接下来我们分别利用普通写法和继承的写法来实现网页中的内容，看一下继承存在的意义以及好处




**普通实现：**

```C++
//Java页面
class Java 
{
public:
	void header()
	{
		cout << "首页、公开课、登录、注册...（公共头部）" << endl;
	}
	void footer()
	{
		cout << "帮助中心、交流合作、站内地图...(公共底部)" << endl;
	}
	void left()
	{
		cout << "Java,Python,C++...(公共分类列表)" << endl;
	}
	void content()
	{
		cout << "JAVA学科视频" << endl;
	}
};
//Python页面
class Python
{
public:
	void header()
	{
		cout << "首页、公开课、登录、注册...（公共头部）" << endl;
	}
	void footer()
	{
		cout << "帮助中心、交流合作、站内地图...(公共底部)" << endl;
	}
	void left()
	{
		cout << "Java,Python,C++...(公共分类列表)" << endl;
	}
	void content()
	{
		cout << "Python学科视频" << endl;
	}
};
//C++页面
class CPP 
{
public:
	void header()
	{
		cout << "首页、公开课、登录、注册...（公共头部）" << endl;
	}
	void footer()
	{
		cout << "帮助中心、交流合作、站内地图...(公共底部)" << endl;
	}
	void left()
	{
		cout << "Java,Python,C++...(公共分类列表)" << endl;
	}
	void content()
	{
		cout << "C++学科视频" << endl;
	}
};

void test01()
{
	//Java页面
	cout << "Java下载视频页面如下： " << endl;
	Java ja;
	ja.header();
	ja.footer();
	ja.left();
	ja.content();
	cout << "--------------------" << endl;

	//Python页面
	cout << "Python下载视频页面如下： " << endl;
	Python py;
	py.header();
	py.footer();
	py.left();
	py.content();
	cout << "--------------------" << endl;

	//C++页面
	cout << "C++下载视频页面如下： " << endl;
	CPP cp;
	cp.header();
	cp.footer();
	cp.left();
	cp.content();

}

int main() {

	test01();

	system("pause");

	return 0;
}
```



**继承实现：**

```C++
//公共页面
class BasePage
{
public:
	void header()
	{
		cout << "首页、公开课、登录、注册...（公共头部）" << endl;
	}

	void footer()
	{
		cout << "帮助中心、交流合作、站内地图...(公共底部)" << endl;
	}
	void left()
	{
		cout << "Java,Python,C++...(公共分类列表)" << endl;
	}

};

//Java页面
class Java : public BasePage
{
public:
	void content()
	{
		cout << "JAVA学科视频" << endl;
	}
};
//Python页面
class Python : public BasePage
{
public:
	void content()
	{
		cout << "Python学科视频" << endl;
	}
};
//C++页面
class CPP : public BasePage
{
public:
	void content()
	{
		cout << "C++学科视频" << endl;
	}
};

void test01()
{
	//Java页面
	cout << "Java下载视频页面如下： " << endl;
	Java ja;
	ja.header();
	ja.footer();
	ja.left();
	ja.content();
	cout << "--------------------" << endl;

	//Python页面
	cout << "Python下载视频页面如下： " << endl;
	Python py;
	py.header();
	py.footer();
	py.left();
	py.content();
	cout << "--------------------" << endl;

	//C++页面
	cout << "C++下载视频页面如下： " << endl;
	CPP cp;
	cp.header();
	cp.footer();
	cp.left();
	cp.content();


}

int main() {

	test01();

	system("pause");

	return 0;
}
```



**总结：**

继承的好处：==可以减少重复的代码==

class A : public B; 

A 类称为子类 或 派生类

B 类称为父类 或 基类



**派生类中的成员，包含两大部分**：

一类是从基类继承过来的，一类是自己增加的成员。

从基类继承过过来的表现其共性，而新增的成员体现了其个性。









#### 4.6.2 继承方式



继承的语法：`class 子类 : 继承方式  父类`



**继承方式一共有三种：**

* 公共继承
* 保护继承
* 私有继承





![img](assets/clip_image002.png)





**示例：**

```C++
class Base1
{
public: 
	int m_A;
protected:
	int m_B;
private:
	int m_C;
};

//公共继承
class Son1 :public Base1
{
public:
	void func()
	{
		m_A; //可访问 public权限
		m_B; //可访问 protected权限
		//m_C; //不可访问
	}
};

void myClass()
{
	Son1 s1;
	s1.m_A; //其他类只能访问到公共权限
}

//保护继承
class Base2
{
public:
	int m_A;
protected:
	int m_B;
private:
	int m_C;
};
class Son2:protected Base2
{
public:
	void func()
	{
		m_A; //可访问 protected权限
		m_B; //可访问 protected权限
		//m_C; //不可访问
	}
};
void myClass2()
{
	Son2 s;
	//s.m_A; //不可访问
}

//私有继承
class Base3
{
public:
	int m_A;
protected:
	int m_B;
private:
	int m_C;
};
class Son3:private Base3
{
public:
	void func()
	{
		m_A; //可访问 private权限
		m_B; //可访问 private权限
		//m_C; //不可访问
	}
};
class GrandSon3 :public Son3
{
public:
	void func()
	{
		//Son3是私有继承，所以继承Son3的属性在GrandSon3中都无法访问到
		//m_A;
		//m_B;
		//m_C;
	}
};
```









#### 4.6.3 继承中的对象模型



**问题：**从父类继承过来的成员，哪些属于子类对象中？



**示例：**

```C++
class Base
{
public:
	int m_A;
protected:
	int m_B;
private:
	int m_C; //私有成员只是被隐藏了，但是还是会继承下去
};

//公共继承
class Son :public Base
{
public:
	int m_D;
};

void test01()
{
	cout << "sizeof Son = " << sizeof(Son) << endl;
}

int main() {

	test01();

	system("pause");

	return 0;
}
```





利用工具查看：



![1545881904150](assets/1545881904150.png)



打开工具窗口后，定位到当前CPP文件的盘符

然后输入： cl /d1 reportSingleClassLayout查看的类名   所属文件名



效果如下图：



![1545882158050](assets/1545882158050.png)



> 结论： 父类中私有成员也是被子类继承下去了，只是由编译器给隐藏后访问不到



















#### 4.6.4 继承中构造和析构顺序



子类继承父类后，当创建子类对象，也会调用父类的构造函数



问题：父类和子类的构造和析构顺序是谁先谁后？



**示例：**

```C++
class Base 
{
public:
	Base()
	{
		cout << "Base构造函数!" << endl;
	}
	~Base()
	{
		cout << "Base析构函数!" << endl;
	}
};

class Son : public Base
{
public:
	Son()
	{
		cout << "Son构造函数!" << endl;
	}
	~Son()
	{
		cout << "Son析构函数!" << endl;
	}

};


void test01()
{
	//继承中 先调用父类构造函数，再调用子类构造函数，析构顺序与构造相反
	Son s;
}

int main() {

	test01();

	system("pause");

	return 0;
}
```



> 总结：继承中 先调用父类构造函数，再调用子类构造函数，析构顺序与构造相反











#### 4.6.5 继承同名成员处理方式



问题：当子类与父类出现同名的成员，如何通过子类对象，访问到子类或父类中同名的数据呢？



* 访问子类同名成员   直接访问即可
* 访问父类同名成员   需要加作用域



**示例：**

```C++
class Base {
public:
	Base()
	{
		m_A = 100;
	}

	void func()
	{
		cout << "Base - func()调用" << endl;
	}

	void func(int a)
	{
		cout << "Base - func(int a)调用" << endl;
	}

public:
	int m_A;
};


class Son : public Base {
public:
	Son()
	{
		m_A = 200;
	}

	//当子类与父类拥有同名的成员函数，子类会隐藏父类中所有版本的同名成员函数
	//如果想访问父类中被隐藏的同名成员函数，需要加父类的作用域
	void func()
	{
		cout << "Son - func()调用" << endl;
	}
public:
	int m_A;
};

void test01()
{
	Son s;

	cout << "Son下的m_A = " << s.m_A << endl;
	cout << "Base下的m_A = " << s.Base::m_A << endl;

	s.func();
	s.Base::func();
	s.Base::func(10);

}
int main() {

	test01();

	system("pause");
	return EXIT_SUCCESS;
}
```

总结：

1. 子类对象可以直接访问到子类中同名成员
2. 子类对象加作用域可以访问到父类同名成员
3. 当子类与父类拥有同名的成员函数，子类会隐藏父类中同名成员函数，加作用域可以访问到父类中同名函数













#### 4.6.6 继承同名静态成员处理方式



问题：继承中同名的静态成员在子类对象上如何进行访问？



静态成员和非静态成员出现同名，处理方式一致



- 访问子类同名成员   直接访问即可
- 访问父类同名成员   需要加作用域



**示例：**

```C++
class Base {
public:
	static void func()
	{
		cout << "Base - static void func()" << endl;
	}
	static void func(int a)
	{
		cout << "Base - static void func(int a)" << endl;
	}

	static int m_A;
};

int Base::m_A = 100;

class Son : public Base {
public:
	static void func()
	{
		cout << "Son - static void func()" << endl;
	}
	static int m_A;
};

int Son::m_A = 200;

//同名成员属性
void test01()
{
	//通过对象访问
	cout << "通过对象访问： " << endl;
	Son s;
	cout << "Son  下 m_A = " << s.m_A << endl;
	cout << "Base 下 m_A = " << s.Base::m_A << endl;

	//通过类名访问
	cout << "通过类名访问： " << endl;
	cout << "Son  下 m_A = " << Son::m_A << endl;
	cout << "Base 下 m_A = " << Son::Base::m_A << endl;
}

//同名成员函数
void test02()
{
	//通过对象访问
	cout << "通过对象访问： " << endl;
	Son s;
	s.func();
	s.Base::func();

	cout << "通过类名访问： " << endl;
	Son::func();
	Son::Base::func();
	//出现同名，子类会隐藏掉父类中所有同名成员函数，需要加作作用域访问
	Son::Base::func(100);
}
int main() {

	//test01();
	test02();

	system("pause");

	return 0;
}
```

> 总结：同名静态成员处理方式和非静态处理方式一样，只不过有两种访问的方式（通过对象 和 通过类名）













#### 4.6.7 多继承语法



C++允许**一个类继承多个类**



语法：` class 子类 ：继承方式 父类1 ， 继承方式 父类2...`



多继承可能会引发父类中有同名成员出现，需要加作用域区分



**C++实际开发中不建议用多继承**







**示例：**

```C++
class Base1 {
public:
	Base1()
	{
		m_A = 100;
	}
public:
	int m_A;
};

class Base2 {
public:
	Base2()
	{
		m_A = 200;  //开始是m_B 不会出问题，但是改为mA就会出现不明确
	}
public:
	int m_A;
};

//语法：class 子类：继承方式 父类1 ，继承方式 父类2 
class Son : public Base2, public Base1 
{
public:
	Son()
	{
		m_C = 300;
		m_D = 400;
	}
public:
	int m_C;
	int m_D;
};


//多继承容易产生成员同名的情况
//通过使用类名作用域可以区分调用哪一个基类的成员
void test01()
{
	Son s;
	cout << "sizeof Son = " << sizeof(s) << endl;
	cout << s.Base1::m_A << endl;
	cout << s.Base2::m_A << endl;
}

int main() {

	test01();

	system("pause");

	return 0;
}
```



> 总结： 多继承中如果父类中出现了同名情况，子类使用时候要加作用域











#### 4.6.8 菱形继承



**菱形继承概念：**

​	两个派生类继承同一个基类

​	又有某个类同时继承者两个派生类

​	这种继承被称为菱形继承，或者钻石继承



**典型的菱形继承案例：**



![IMG_256](assets/clip_image002.jpg)



**菱形继承问题：**



1.     羊继承了动物的数据，驼同样继承了动物的数据，当草泥马使用数据时，就会产生二义性。

2. 草泥马继承自动物的数据继承了两份，其实我们应该清楚，这份数据我们只需要一份就可以。



**示例：**

```C++
class Animal
{
public:
	int m_Age;
};

//继承前加virtual关键字后，变为虚继承
//此时公共的父类Animal称为虚基类
class Sheep : virtual public Animal {};
class Tuo   : virtual public Animal {};// 共享同一个 Animal 实例
class SheepTuo : public Sheep, public Tuo {};

void test01()
{
	SheepTuo st;
	st.Sheep::m_Age = 100;
	st.Tuo::m_Age = 200;

	cout << "st.Sheep::m_Age = " << st.Sheep::m_Age << endl;
	cout << "st.Tuo::m_Age = " <<  st.Tuo::m_Age << endl;
	cout << "st.m_Age = " << st.m_Age << endl;
}


int main() {

	test01();

	system("pause");

	return 0;
}
```



总结：

* 菱形继承带来的主要问题是子类继承两份相同的数据，导致资源浪费以及毫无意义
* 利用虚继承可以解决菱形继承问题



















### 4.7  多态

#### 4.7.1 多态的基本概念



**多态是C++面向对象三大特性之一**

多态分为两类

* 静态多态: 函数重载 和 运算符重载属于静态多态，复用函数名
* 动态多态: 派生类和虚函数实现运行时多态



静态多态和动态多态区别：

* 静态多态的函数地址早绑定  -  编译阶段确定函数地址
* 动态多态的函数地址晚绑定  -  运行阶段确定函数地址



下面通过案例进行讲解多态



```C++
class Animal
{
public:
	//Speak函数就是虚函数
	//函数前面加上virtual关键字，变成虚函数，那么编译器在编译的时候就不能确定函数调用了。
	virtual void speak()
	{
		cout << "动物在说话" << endl;
	}
};

class Cat :public Animal
{
public:
	void speak()
	{
		cout << "小猫在说话" << endl;
	}
};

class Dog :public Animal
{
public:

	void speak()
	{
		cout << "小狗在说话" << endl;
	}

};
//我们希望传入什么对象，那么就调用什么对象的函数
//如果函数地址在编译阶段就能确定，那么静态联编
//如果函数地址在运行阶段才能确定，就是动态联编

void DoSpeak(Animal & animal)
{
	animal.speak();
}
//
//多态满足条件： 
//1、有继承关系
//2、子类重写父类中的虚函数
//多态使用：
//父类指针或引用指向子类对象

void test01()
{
	Cat cat;
	DoSpeak(cat);


	Dog dog;
	DoSpeak(dog);
}


int main() {

	test01();

	system("pause");

	return 0;
}
```

总结：

多态满足条件

* 有继承关系
* 子类重写父类中的虚函数

多态使用条件

* 父类指针或引用指向子类对象

重写：函数返回值类型  函数名 参数列表 完全一致称为重写









#### 4.7.2 多态案例一-计算器类



案例描述：

分别利用普通写法和多态技术，设计实现两个操作数进行运算的计算器类



多态的优点：

* 代码组织结构清晰
* 可读性强
* 利于前期和后期的扩展以及维护



**示例：**

```C++
//普通实现
class Calculator {
public:
	int getResult(string oper)
	{
		if (oper == "+") {
			return m_Num1 + m_Num2;
		}
		else if (oper == "-") {
			return m_Num1 - m_Num2;
		}
		else if (oper == "*") {
			return m_Num1 * m_Num2;
		}
		//如果要提供新的运算，需要修改源码
	}
public:
	int m_Num1;
	int m_Num2;
};

void test01()
{
	//普通实现测试
	Calculator c;
	c.m_Num1 = 10;
	c.m_Num2 = 10;
	cout << c.m_Num1 << " + " << c.m_Num2 << " = " << c.getResult("+") << endl;

	cout << c.m_Num1 << " - " << c.m_Num2 << " = " << c.getResult("-") << endl;

	cout << c.m_Num1 << " * " << c.m_Num2 << " = " << c.getResult("*") << endl;
}



//多态实现
//抽象计算器类
//多态优点：代码组织结构清晰，可读性强，利于前期和后期的扩展以及维护
class AbstractCalculator
{
public :

	virtual int getResult()
	{
		return 0;
	}

	int m_Num1;
	int m_Num2;
};

//加法计算器
class AddCalculator :public AbstractCalculator
{
public:
	int getResult()
	{
		return m_Num1 + m_Num2;
	}
};

//减法计算器
class SubCalculator :public AbstractCalculator
{
public:
	int getResult()
	{
		return m_Num1 - m_Num2;
	}
};

//乘法计算器
class MulCalculator :public AbstractCalculator
{
public:
	int getResult()
	{
		return m_Num1 * m_Num2;
	}
};


void test02()
{
	//创建加法计算器
	AbstractCalculator *abc = new AddCalculator;
	abc->m_Num1 = 10;
	abc->m_Num2 = 10;
	cout << abc->m_Num1 << " + " << abc->m_Num2 << " = " << abc->getResult() << endl;
	delete abc;  //用完了记得销毁

	//创建减法计算器
	abc = new SubCalculator;
	abc->m_Num1 = 10;
	abc->m_Num2 = 10;
	cout << abc->m_Num1 << " - " << abc->m_Num2 << " = " << abc->getResult() << endl;
	delete abc;  

	//创建乘法计算器
	abc = new MulCalculator;
	abc->m_Num1 = 10;
	abc->m_Num2 = 10;
	cout << abc->m_Num1 << " * " << abc->m_Num2 << " = " << abc->getResult() << endl;
	delete abc;
}

int main() {

	//test01();

	test02();

	system("pause");

	return 0;
}
```

> 总结：C++开发提倡利用多态设计程序架构，因为多态优点很多

















#### 4.7.3 纯虚函数和抽象类



在多态中，通常父类中虚函数的实现是毫无意义的，主要都是调用子类重写的内容



因此可以将虚函数改为**纯虚函数**



纯虚函数语法：`virtual 返回值类型 函数名 （参数列表）= 0 ;`



当类中有了纯虚函数，这个类也称为==抽象类==



**抽象类特点**：

 * 无法实例化对象
 * 子类必须重写抽象类中的纯虚函数，否则也属于抽象类





**示例：**

```C++
class Base
{
public:
	//纯虚函数
	//类中只要有一个纯虚函数就称为抽象类
	//抽象类无法实例化对象
	//子类必须重写父类中的纯虚函数，否则也属于抽象类
	virtual void func() = 0;
};

class Son :public Base
{
public:
	virtual void func() 
	{
		cout << "func调用" << endl;
	};
};

void test01()
{
	Base * base = NULL;
	//base = new Base; // 错误，抽象类无法实例化对象
	base = new Son;
	base->func();
	delete base;//记得销毁
}

int main() {

	test01();

	system("pause");

	return 0;
}
```















#### 4.7.4 多态案例二-制作饮品

**案例描述：**

制作饮品的大致流程为：煮水 -  冲泡 - 倒入杯中 - 加入辅料



利用多态技术实现本案例，提供抽象制作饮品基类，提供子类制作咖啡和茶叶



![1545985945198](assets/1545985945198.png)



**示例：**

```C++
//抽象制作饮品
class AbstractDrinking {
public:
	//烧水
	virtual void Boil() = 0;
	//冲泡
	virtual void Brew() = 0;
	//倒入杯中
	virtual void PourInCup() = 0;
	//加入辅料
	virtual void PutSomething() = 0;
	//规定流程
	void MakeDrink() {
		Boil();
		Brew();
		PourInCup();
		PutSomething();
	}
};

//制作咖啡
class Coffee : public AbstractDrinking {
public:
	//烧水
	virtual void Boil() {
		cout << "煮农夫山泉!" << endl;
	}
	//冲泡
	virtual void Brew() {
		cout << "冲泡咖啡!" << endl;
	}
	//倒入杯中
	virtual void PourInCup() {
		cout << "将咖啡倒入杯中!" << endl;
	}
	//加入辅料
	virtual void PutSomething() {
		cout << "加入牛奶!" << endl;
	}
};

//制作茶水
class Tea : public AbstractDrinking {
public:
	//烧水
	virtual void Boil() {
		cout << "煮自来水!" << endl;
	}
	//冲泡
	virtual void Brew() {
		cout << "冲泡茶叶!" << endl;
	}
	//倒入杯中
	virtual void PourInCup() {
		cout << "将茶水倒入杯中!" << endl;
	}
	//加入辅料
	virtual void PutSomething() {
		cout << "加入枸杞!" << endl;
	}
};

//业务函数
void DoWork(AbstractDrinking* drink) {
	drink->MakeDrink();
	delete drink;
}

void test01() {
	DoWork(new Coffee);
	cout << "--------------" << endl;
	DoWork(new Tea);
}


int main() {

	test01();

	system("pause");

	return 0;
}
```



















#### 4.7.5 虚析构和纯虚析构



多态使用时，如果子类中有属性开辟到堆区，那么父类指针在释放时无法调用到子类的析构代码



解决方式：将父类中的析构函数改为**虚析构**或者**纯虚析构**



虚析构和纯虚析构共性：

* 可以解决父类指针释放子类对象
* 都需要有具体的函数实现

虚析构和纯虚析构区别：

* 如果是纯虚析构，该类属于抽象类，无法实例化对象



虚析构语法：

`virtual ~类名(){}`

纯虚析构语法：

` virtual ~类名() = 0;`

`类名::~类名(){}`



**示例：**

```C++
class Animal {
public:

	Animal()
	{
		cout << "Animal 构造函数调用！" << endl;
	}
	virtual void Speak() = 0;

	//析构函数加上virtual关键字，变成虚析构函数
	//virtual ~Animal()
	//{
	//	cout << "Animal虚析构函数调用！" << endl;
	//}


	virtual ~Animal() = 0;
};

Animal::~Animal()
{
	cout << "Animal 纯虚析构函数调用！" << endl;
}

//和包含普通纯虚函数的类一样，包含了纯虚析构函数的类也是一个抽象类。不能够被实例化。

class Cat : public Animal {
public:
	Cat(string name)
	{
		cout << "Cat构造函数调用！" << endl;
		m_Name = new string(name);
	}
	virtual void Speak()
	{
		cout << *m_Name <<  "小猫在说话!" << endl;
	}
	~Cat()
	{
		cout << "Cat析构函数调用!" << endl;
		if (this->m_Name != NULL) {
			delete m_Name;
			m_Name = NULL;
		}
	}

public:
	string *m_Name;
};

void test01()
{
	Animal *animal = new Cat("Tom");
	animal->Speak();

	//通过父类指针去释放，会导致子类对象可能清理不干净，造成内存泄漏
	//怎么解决？给基类增加一个虚析构函数
	//虚析构函数就是用来解决通过父类指针释放子类对象
	delete animal;
}

int main() {

	test01();

	system("pause");

	return 0;
}
```



总结：

​	1. 虚析构或纯虚析构就是用来解决通过父类指针释放子类对象

​	2. 如果子类中没有堆区数据，可以不写为虚析构或纯虚析构

​	3. 拥有纯虚析构函数的类也属于抽象类















#### 4.7.6 多态案例三-电脑组装



**案例描述：**



电脑主要组成部件为 CPU（用于计算），显卡（用于显示），内存条（用于存储）

将每个零件封装出抽象基类，并且提供不同的厂商生产不同的零件，例如Intel厂商和Lenovo厂商

创建电脑类提供让电脑工作的函数，并且调用每个零件工作的接口

测试时组装三台不同的电脑进行工作





**示例：**

```C++
#include<iostream>
using namespace std;

//抽象CPU类
class CPU
{
public:
	//抽象的计算函数
	virtual void calculate() = 0;
};

//抽象显卡类
class VideoCard
{
public:
	//抽象的显示函数
	virtual void display() = 0;
};

//抽象内存条类
class Memory
{
public:
	//抽象的存储函数
	virtual void storage() = 0;
};

//电脑类
class Computer
{
public:
	Computer(CPU * cpu, VideoCard * vc, Memory * mem)
	{
		m_cpu = cpu;
		m_vc = vc;
		m_mem = mem;
	}

	//提供工作的函数
	void work()
	{
		//让零件工作起来，调用接口
		m_cpu->calculate();

		m_vc->display();

		m_mem->storage();
	}

	//提供析构函数 释放3个电脑零件
	~Computer()
	{

		//释放CPU零件
		if (m_cpu != NULL)
		{
			delete m_cpu;
			m_cpu = NULL;
		}

		//释放显卡零件
		if (m_vc != NULL)
		{
			delete m_vc;
			m_vc = NULL;
		}

		//释放内存条零件
		if (m_mem != NULL)
		{
			delete m_mem;
			m_mem = NULL;
		}
	}

private:

	CPU * m_cpu; //CPU的零件指针
	VideoCard * m_vc; //显卡零件指针
	Memory * m_mem; //内存条零件指针
};

//具体厂商
//Intel厂商
class IntelCPU :public CPU
{
public:
	virtual void calculate()
	{
		cout << "Intel的CPU开始计算了！" << endl;
	}
};

class IntelVideoCard :public VideoCard
{
public:
	virtual void display()
	{
		cout << "Intel的显卡开始显示了！" << endl;
	}
};

class IntelMemory :public Memory
{
public:
	virtual void storage()
	{
		cout << "Intel的内存条开始存储了！" << endl;
	}
};

//Lenovo厂商
class LenovoCPU :public CPU
{
public:
	virtual void calculate()
	{
		cout << "Lenovo的CPU开始计算了！" << endl;
	}
};

class LenovoVideoCard :public VideoCard
{
public:
	virtual void display()
	{
		cout << "Lenovo的显卡开始显示了！" << endl;
	}
};

class LenovoMemory :public Memory
{
public:
	virtual void storage()
	{
		cout << "Lenovo的内存条开始存储了！" << endl;
	}
};


void test01()
{
	//第一台电脑零件
	CPU * intelCpu = new IntelCPU;
	VideoCard * intelCard = new IntelVideoCard;
	Memory * intelMem = new IntelMemory;

	cout << "第一台电脑开始工作：" << endl;
	//创建第一台电脑
	Computer * computer1 = new Computer(intelCpu, intelCard, intelMem);
	computer1->work();
	delete computer1;

	cout << "-----------------------" << endl;
	cout << "第二台电脑开始工作：" << endl;
	//第二台电脑组装
	Computer * computer2 = new Computer(new LenovoCPU, new LenovoVideoCard, new LenovoMemory);;
	computer2->work();
	delete computer2;

	cout << "-----------------------" << endl;
	cout << "第三台电脑开始工作：" << endl;
	//第三台电脑组装
	Computer * computer3 = new Computer(new LenovoCPU, new IntelVideoCard, new LenovoMemory);;
	computer3->work();
	delete computer3;

}
```













## 5 文件操作



程序运行时产生的数据都属于临时数据，程序一旦运行结束都会被释放

通过**文件可以将数据持久化**

C++中对文件操作需要包含头文件 ==&lt; fstream &gt;==



文件类型分为两种：

1. **文本文件**     -  文件以文本的**ASCII码**形式存储在计算机中
2. **二进制文件** -  文件以文本的**二进制**形式存储在计算机中，用户一般不能直接读懂它们



操作文件的三大类:

1. ofstream：写操作
2. ifstream： 读操作
3. fstream ： 读写操作



### 5.1文本文件

#### 5.1.1写文件

   写文件步骤如下：

1. 包含头文件   

     \#include <fstream\>

2. 创建流对象  

   ofstream ofs;

3. 打开文件

   ofs.open("文件路径",打开方式);

4. 写数据

   ofs << "写入的数据";

5. 关闭文件

   ofs.close();

   ​

文件打开方式：

| 打开方式    | 解释                       |
| ----------- | -------------------------- |
| ios::in     | 为读文件而打开文件         |
| ios::out    | 为写文件而打开文件         |
| ios::ate    | 初始位置：文件尾           |
| ios::app    | 追加方式写文件             |
| ios::trunc  | 如果文件存在先删除，再创建 |
| ios::binary | 二进制方式                 |

**注意：** 文件打开方式可以配合使用，利用|操作符

**例如：**用二进制方式写文件 `ios::binary |  ios:: out`





**示例：**

```C++
#include <fstream>

void test01()
{
	ofstream ofs;
	ofs.open("test.txt", ios::out);

	ofs << "姓名：张三" << endl;
	ofs << "性别：男" << endl;
	ofs << "年龄：18" << endl;

	ofs.close();
}

int main() {

	test01();

	system("pause");

	return 0;
}
```

总结：

* 文件操作必须包含头文件 fstream
* 读文件可以利用 ofstream  ，或者fstream类
* 打开文件时候需要指定操作文件的路径，以及打开方式
* 利用<<可以向文件中写数据
* 操作完毕，要关闭文件

















#### 5.1.2读文件



读文件与写文件步骤相似，但是读取方式相对于比较多



读文件步骤如下：

1. 包含头文件   

     \#include <fstream\>

2. 创建流对象  

   ifstream ifs;

3. 打开文件并判断文件是否打开成功

   ifs.open("文件路径",打开方式);

4. 读数据

   四种方式读取

5. 关闭文件

   ifs.close();



**示例：**

```C++
#include <fstream>
#include <string>
void test01()
{
	ifstream ifs;
	ifs.open("test.txt", ios::in);

	if (!ifs.is_open())
	{
		cout << "文件打开失败" << endl;
		return;
	}

	//第一种方式
	//char buf[1024] = { 0 };
	//while (ifs >> buf)
	//{
	//	cout << buf << endl;
	//}

	//第二种
	//char buf[1024] = { 0 };
	//while (ifs.getline(buf,sizeof(buf)))
	//{
	//	cout << buf << endl;
	//}

	//第三种
	//string buf;
	//while (getline(ifs, buf))
	//{
	//	cout << buf << endl;
	//}

	char c;
	while ((c = ifs.get()) != EOF)
	{
		cout << c;
	}

	ifs.close();


}

int main() {

	test01();

	system("pause");

	return 0;
}
```

总结：

- 读文件可以利用 ifstream  ，或者fstream类
- 利用is_open函数可以判断文件是否打开成功
- close 关闭文件 















### 5.2 二进制文件

以二进制的方式对文件进行读写操作

打开方式要指定为 ==ios::binary==



#### 5.2.1 写文件

二进制方式写文件主要利用流对象调用成员函数write

函数原型 ：`ostream& write(const char * buffer,int len);`

参数解释：字符指针buffer指向内存中一段存储空间。len是读写的字节数



**示例：**

```C++
#include <fstream>
#include <string>

class Person
{
public:
	char m_Name[64];
	int m_Age;
};

//二进制文件  写文件
void test01()
{
	//1、包含头文件

	//2、创建输出流对象
	ofstream ofs("person.txt", ios::out | ios::binary);
	
	//3、打开文件
	//ofs.open("person.txt", ios::out | ios::binary);

	Person p = {"张三"  , 18};

	//4、写文件
	ofs.write((const char *)&p, sizeof(p));

	//5、关闭文件
	ofs.close();
}

int main() {

	test01();

	system("pause");

	return 0;
}
```

总结：

* 文件输出流对象 可以通过write函数，以二进制方式写数据











#### 5.2.2 读文件

二进制方式读文件主要利用流对象调用成员函数read

函数原型：`istream& read(char *buffer,int len);`

参数解释：字符指针buffer指向内存中一段存储空间。len是读写的字节数

示例：

```C++
#include <fstream>
#include <string>

class Person
{
public:
	char m_Name[64];
	int m_Age;
};

void test01()
{
	ifstream ifs("person.txt", ios::in | ios::binary);
	if (!ifs.is_open())
	{
		cout << "文件打开失败" << endl;
	}

	Person p;
	ifs.read((char *)&p, sizeof(p));

	cout << "姓名： " << p.m_Name << " 年龄： " << p.m_Age << endl;
}

int main() {

	test01();

	system("pause");

	return 0;
}
```



- 文件输入流对象 可以通过read函数，以二进制方式读数据


---

# 地轨项目 C++ 核心编程补充篇

> 本篇是给后续接手地轨焊接项目的新同学看的。前面的内容讲的是 C++ 基础语法，本篇把这些语法放回到本项目里，说明它们为什么会出现、解决什么问题、该怎么读、该怎么改。
>
> 地轨项目不是一个单纯算法 demo，而是一个真实设备控制软件：Qt 负责界面和事件系统，OpenCV 负责图像，PCL/VTK 负责点云和显示，TensorRT/CUDA 负责深度学习推理，modbus 和机器人 SDK/TCP 负责设备通讯，`RailWeldingSystem` 负责把所有模块组织成自动焊接流程。

## 1 新人先看懂项目整体

### 1.1 项目入口

程序入口在：

```cpp
src/main.cpp
```

主流程大概是：

```cpp
int main(int argc, char *argv[]) {
    QApplication a(argc, argv);

    vtkOutputWindow::SetGlobalWarningDisplay(0);
    CrashHandler::Init(L"data/debug");
    initPlog();
    registerMetaType();

    RailWeldingMainWindow w;
    w.show();
    return a.exec();
}
```

新人要先理解这几件事：

- `QApplication` 是 Qt GUI 程序的核心对象。
- `a.exec()` 进入 Qt 事件循环，后面按钮点击、线程信号、定时器触发都靠事件循环分发。
- `initPlog()` 初始化日志。
- `CrashHandler::Init(...)` 初始化崩溃转储。
- `registerMetaType()` 注册跨线程信号能传递的复杂类型，例如 `cv::Mat`、点云指针、机器人位姿等。
- `RailWeldingMainWindow` 是主界面，负责显示和用户交互。

### 1.2 核心控制类

项目中最重要的业务调度类是：

```cpp
src/weldingSystem/RailWeldingSystem.h
src/weldingSystem/RailWeldingSystem.cpp
```

它可以理解为“总调度室”。它不直接做所有事情，而是管理和协调这些模块：

- `StructLightCamera`：结构光相机、投影仪、点云重建。
- `SeamDetWithPointCloud`：基于点云的焊缝检测。
- `SeamDetWithSeg`：基于深度学习分割结果的焊缝检测。
- `RobotTrajectoryPlanning`：机器人轨迹规划。
- `AbstractRobot`：机器人抽象接口，下面有安川、宝元等实现。
- `Rail`：地轨 PLC 控制，基于 modbus。
- `WorkpieceCoarseLocalization`：工件粗定位。
- `ErrorSave`：错误数据保存。
- `PhotoPlanner`：拍照位置规划。

可以把主流程理解成：

```text
用户点按钮 / 自动流程触发
        ↓
RailWeldingMainWindow 发出信号
        ↓
RailWeldingSystem 接收信号并调度
        ↓
地轨、机器人、结构光、检测、轨迹规划等模块工作
        ↓
各模块通过信号把结果发回系统或 UI
```

### 1.3 项目目录怎么读

常见目录含义：

```text
src/ui                         主界面和显示控件
src/weldingSystem              系统级调度
src/rail                       地轨 PLC / modbus
src/robotFactory               机器人抽象类、具体机器人和工厂
src/cameraFactory              相机抽象类、Basler 相机和工厂
src/projectFactory             投影仪抽象类、腾聚投影仪和工厂
src/structLightCamera          结构光采集、触发、重建
src/seamDetWithPointCloud      点云焊缝检测
src/seamDetWithSeg             分割焊缝检测
src/deepLearning               TensorRT/CUDA 推理封装
src/robotTrajectoryPlanning    机器人轨迹规划
src/workpieceCoarseLocalization 工件粗定位
src/utils                      公共工具、线程池、图像控件、点云工具
data/config                    json 配置
data/DL_models                 engine / onnx 模型
3rdParty                       第三方库和厂商 SDK
```

工程组织使用 qmake：

```text
ground_rail_welding.pro
src/src.pri
各模块自己的 *.pri
```

`.pro` 和 `.pri` 的作用类似“告诉编译器有哪些源码、头文件、库路径和宏定义”。

## 2 本项目用到的 C++ 基础语法

### 2.1 头文件和源文件

项目里大多数类都有一组 `.h + .cpp`：

```text
Rail.h     声明类、成员变量、函数接口
Rail.cpp   实现函数逻辑
```

头文件常见结构：

```cpp
#ifndef RAIL_H
#define RAIL_H

class Rail : public QObject {
    Q_OBJECT
public:
    Rail(QObject *parent = nullptr);
    ~Rail();
};

#endif
```

这里有几个基础点：

- `#ifndef / #define / #endif` 是头文件保护，防止重复包含。
- `class Rail` 定义类。
- `public / protected / private` 控制访问权限。
- `QObject` 是 Qt 对象基类。
- `Q_OBJECT` 是 Qt 元对象系统需要的宏，使用信号槽时必须加。

### 2.2 类和对象

项目中几乎所有业务都是类：

```cpp
class RailWeldingSystem : public QObject {
    Q_OBJECT
public:
    RailWeldingSystem(QObject *parent = nullptr);
    ~RailWeldingSystem();

private:
    std::shared_ptr<StructLightCamera> structLightCamera{nullptr};
    Rail *rail = nullptr;
};
```

理解方法：

- 类是“图纸”，对象是“真正运行的实例”。
- `RailWeldingSystem` 是系统调度类。
- `StructLightCamera` 是结构光模块对象。
- `Rail *rail` 是地轨对象指针，由外部注入进来。

### 2.3 构造函数和析构函数

构造函数负责对象初始化：

```cpp
RailWeldingSystem(QObject *parent = nullptr);
```

析构函数负责释放资源：

```cpp
~RailWeldingSystem();
```

在设备控制项目里，析构非常重要，因为可能要：

- 停止线程。
- 断开相机。
- 关闭机器人连接。
- 释放 modbus 连接。
- 释放 CUDA/TensorRT 内存。
- 写日志。

### 2.4 指针

项目中常见三类指针：

```cpp
Rail *rail = nullptr;
modbus_t *modbusTcp = nullptr;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
```

含义：

- `Rail *` 是普通 C++ 指针，生命周期要自己明确。
- `modbus_t *` 是 C 库指针，要用 `modbus_free` 释放。
- `pcl::PointCloud<...>::Ptr` 是 PCL 的智能指针类型，本质上类似 `shared_ptr`。

新手要记住：

```cpp
if (ptr == nullptr) {
    return;
}
```

设备、点云、图像、模型指针在使用前都要判断是否有效。

### 2.5 引用

引用常用于函数参数，避免拷贝：

```cpp
void inference(cv::Mat &img, std::vector<SegResult> &res);
```

含义：

- `cv::Mat &img` 表示函数可以直接操作外部传进来的图像。
- `std::vector<SegResult> &res` 表示结果写到外部容器里。

如果不希望函数修改参数，应使用 `const 引用`：

```cpp
void draw(const cv::Mat &image);
```

规则：

- 大对象优先用 `const T&` 传参。
- 需要修改输出时用 `T&`。
- 小对象例如 `int/double/bool` 直接按值传递即可。

### 2.6 const

`const` 表示“不允许修改”。

项目中常见：

```cpp
std::vector<PhotoPlanItem> plan(
    const std::vector<cv::Rect_<double>>& roughRegions,
    const std::vector<int>& mergeLabels);
```

这表示 `plan` 函数只读取 `roughRegions` 和 `mergeLabels`，不会改它们。

成员函数后面的 `const` 表示这个函数不修改当前对象：

```cpp
bool isFullyCovered(const cv::Rect_<double>& outer,
                    const cv::Rect_<double>& inner) const;
```

### 2.7 enum 枚举

枚举用来表达有限状态，避免魔法数字。

项目中有：

```cpp
enum WELD_MODE {
    AUTO_WELD,
    MANUAL_WELD
};

enum WELDING_STAGE {
    PREPARE_STAGE,
    PRELIMINARY_PHOTO_AREA,
    LAST_PHOTO_AREA
};
```

再比如焊缝类型：

```cpp
enum WELD_TYPE {
    BACK_CORNER_BUTT,
    FRONT_CORNER_BUTT,
    BACK_BEAM_BUTT,
    FRONT_BEAM_BUTT,
    FRONT_HORIZONTAL_FILLET,
    FRONT_VERTICAL_FILLET,
    FOOT_HORIZONTAL_FILLET,
    FOOT_VERTICAL_FILLET
};
```

新人读代码时看到枚举，应该马上想到：“这是状态或类型，不是随便的数字。”

### 2.8 继承和多态

本项目大量使用抽象基类：

```cpp
class AbstractCamera : public QObject {
    Q_OBJECT
public slots:
    virtual bool open() = 0;
    virtual bool start() = 0;
    virtual void close() = 0;
};
```

`= 0` 表示纯虚函数。包含纯虚函数的类不能直接创建对象，只能由子类实现。

例如：

```cpp
class BaslerCamera : public AbstractCamera {
public slots:
    bool open() override;
    bool start() override;
    void close() override;
};
```

意义：

- 上层只关心“相机能打开、采图、关闭”。
- 不关心具体是 Basler 还是别的品牌。
- 后续换相机时，只要实现同样接口，上层代码少改。

机器人、投影仪也一样：

```cpp
AbstractRobot
AbstractProjector
AbstractCamera
AbstractSegment
AbstractObjectDetect
AbstractSeamDet
```

### 2.9 override

`override` 表示“我正在重写父类虚函数”。

```cpp
std::shared_ptr<AbstractRobot> createRobot() override;
```

好处：

- 如果函数名、参数、返回值写错，编译器会报错。
- 能避免“以为重写了，实际没重写”的问题。

新代码里重写虚函数时都建议加 `override`。

### 2.10 friend

项目中有：

```cpp
friend class RailWeldingMainWindow;
friend class StructLightCamera;
friend class SettingWidget;
```

`friend` 表示允许某个类访问当前类的私有成员。

优点是方便，缺点是会增加耦合。新人要谨慎使用，只有在 Qt UI 调参或模块确实需要访问内部状态时才考虑。

### 2.11 智能指针

项目主要使用：

```cpp
std::shared_ptr<T>
std::make_shared<T>()
```

例子：

```cpp
std::shared_ptr<PhotoPlanner> photoPlanner =
    std::make_shared<PhotoPlanner>(photoRangeWidth, photoRangeHeight);
```

含义：

- 多个地方可以共享这个对象。
- 最后一个 `shared_ptr` 销毁时，对象自动释放。
- 比裸 `new/delete` 安全。

焊缝信息里也大量使用：

```cpp
std::vector<std::shared_ptr<WeldSeamInfo>> weldAreaInfo;
```

为什么用智能指针：

- 焊缝信息很复杂，包含图像、点云、端点、机器人位姿。
- 在多个模块和线程间传递时，不希望频繁深拷贝。
- 但也不能让对象提前释放。

注意：

- `shared_ptr` 不是万能的，循环引用会导致释放不了。
- 跨线程共享数据时仍然要考虑线程安全。
- 如果对象只有一个拥有者，优先考虑 `unique_ptr`，但本项目主要沿用了 `shared_ptr`。

### 2.12 STL 容器

项目常用容器：

```cpp
std::vector<T>      动态数组
std::queue<T>       队列
std::deque<T>       双端队列
std::map<K, V>      有序映射
std::unordered_map<K, V> 哈希映射
std::set<T>         集合
std::array<T, N>    固定大小数组
```

例子：

```cpp
std::vector<cv::Mat> images;
std::vector<std::shared_ptr<WeldSeamInfo>> seams;
std::queue<ThreadPool::Task> tasks;
```

新人要重点掌握：

- `push_back` 添加元素。
- `emplace_back` 原地构造元素。
- `size()` 获取数量。
- `empty()` 判断是否为空。
- `clear()` 清空。
- 范围 for：

```cpp
for (auto &item : items) {
    // 使用 item
}
```

### 2.13 auto

`auto` 让编译器自动推导类型。

```cpp
auto future = pool.addTask(func);
auto seamInfo = std::make_shared<WeldSeamInfo>();
```

适合类型很长时使用，例如迭代器、智能指针、模板返回值。

但新人不要滥用。读代码时如果 `auto` 让类型不明显，就把鼠标放到 IDE 上看真实类型。

### 2.14 lambda 表达式

线程池里有典型 lambda：

```cpp
workers.emplace_back([this] {
    while (true) {
        Task task;
        // 等待任务并执行
    }
});
```

`[this]` 表示 lambda 可以访问当前对象成员。

Qt 中也常见：

```cpp
connect(timer, &QTimer::timeout, this, [this]() {
    // 定时执行逻辑
});
```

新人理解 lambda 的方式：

```text
lambda = 临时写在函数里的小函数
```

### 2.15 模板

线程池的 `addTask` 是模板函数：

```cpp
template <typename Func, typename... Args>
auto addTask(Func&& func, Args&&... args)
    -> std::future<decltype(func(args...))>;
```

含义：

- `Func` 可以是任意函数、lambda、成员函数包装。
- `Args...` 是任意数量参数。
- 返回值用 `future` 表示将来能拿到结果。

模板的好处是通用，缺点是初学者难读。新人先记住用途：这个函数能把各种任务丢进线程池。

### 2.16 异常

项目里有：

```cpp
throw std::runtime_error("无法找到能覆盖剩余粗定位区域的相机视野");
```

异常表示当前函数无法正常完成。设备控制代码里不要随便抛异常，尤其是跨 Qt 信号槽、线程边界时，要考虑谁捕获、怎么提示 UI、怎么停设备。

### 2.17 文件读写

项目中用到：

```cpp
std::ifstream
std::ofstream
cv::imwrite
pcl::io::savePCDFile
cereal::JSONInputArchive
```

典型用途：

- 保存错误图像。
- 保存错误点云。
- 读取 json 配置。
- 读取机器人路点。
- 保存调试数据。

新手要注意：

- 路径是否存在。
- 文件是否打开成功。
- 中文路径可能带来编码问题。
- 设备调试数据尽量带时间戳。

## 3 Qt 核心机制

### 3.1 QObject

凡是要使用 Qt 信号槽的类，一般继承 `QObject`：

```cpp
class Rail : public QObject {
    Q_OBJECT
public:
    Rail(QObject *parent = nullptr);
};
```

`QObject` 提供：

- 父子对象管理。
- 信号槽。
- 元对象系统。
- 跨线程事件投递。

### 3.2 signals 和 slots

项目中大量写法：

```cpp
signals:
    void sendMessage2Ui(QString message);

public slots:
    void whenAutoWelding();
```

理解方式：

- `signals` 是“我能发出什么消息”。
- `slots` 是“我能响应什么消息”。
- `emit` 是“发出消息”。

例子：

```cpp
emit sendMessage2Ui(u8"结构光相机连接成功");
```

### 3.3 connect

信号和槽通过 `connect` 连接：

```cpp
connect(this,
        &RailWeldingMainWindow::sendAutoWelding,
        railWeldingSystem.get(),
        &RailWeldingSystem::whenAutoWelding);
```

含义：

```text
主界面发出 sendAutoWelding
        ↓
系统类执行 whenAutoWelding
```

新人读 `connect` 时按四段看：

```cpp
connect(发送者, 发送者信号, 接收者, 接收者槽函数);
```

### 3.4 emit

`emit` 后面接信号名：

```cpp
emit sendRailMove2AbsPosition(vel, pos);
```

它不是普通函数调用那么简单。跨线程时，Qt 会把这个调用投递到接收对象所在的线程事件循环里执行。

### 3.5 qRegisterMetaType

跨线程信号传复杂类型时必须注册：

```cpp
qRegisterMetaType<cv::Mat>("cv::Mat");
qRegisterMetaType<pcl::PointCloud<pcl::PointXYZ>::Ptr>(
    "pcl::PointCloud<pcl::PointXYZ>::Ptr");
```

否则 Qt 不知道怎么在事件队列中传递这些类型。

项目里注册了：

- `QImage`
- `cv::Mat`
- `pcl::PointCloud<pcl::PointXYZ>::Ptr`
- `robotPose`
- `robotJointAngle`
- `std::vector<std::shared_ptr<WeldSeamInfo>>`
- 粗定位相关结构体

### 3.6 QObject + QThread

项目推荐用：

```cpp
worker->moveToThread(thread);
thread->start();
```

而不是继承 `QThread` 重写 `run()`。

例子：

```cpp
railWeldingSystem->moveToThread(railWeldingSystemThread);
railWeldingSystemThread->start();
```

意义：

- 对象属于哪个线程，它的槽函数通常就在哪个线程执行。
- UI 留在主线程。
- 相机、检测、机器人、地轨等耗时模块放到工作线程。

重要规则：

- 不要在子线程直接改 UI。
- 子线程要通过信号把数据发回 UI。
- 对象 `moveToThread` 前后要注意创建位置和父对象关系。
- 线程退出时要 `quit()`、`wait()`，避免程序关闭卡死。

### 3.7 QTimer

项目用 `QTimer` 做周期任务，例如地轨状态读取。

```cpp
connect(readRealTimer, &QTimer::timeout, this, &Rail::onRealTimeout);
```

意思是定时器超时后执行 `onRealTimeout()`。

### 3.8 QEventLoop

项目里某些地方用 `QEventLoop` 等待某个异步信号：

```cpp
QEventLoop loop;
connect(this, &Rail::sendSignalFinishWriteRegisters, &loop, &QEventLoop::quit);
loop.exec();
```

这种写法可以把异步操作临时变成“等待完成再继续”。但要小心：

- 等不到信号会卡住。
- 在 UI 线程使用可能造成界面假死。
- 最好配合超时机制。

### 3.9 Qt 图像显示

项目里图像数据通常是 `cv::Mat`，UI 显示可能要转成 `QImage` 或交给封装控件：

```cpp
void RailWeldingMainWindow::whenGetImg2Ui(cv::Mat img) {
    ui->imageWidget->setOpenCVImage(img);
}
```

图像跨线程传递时：

- `cv::Mat` 内部是引用计数。
- 如果后续还会改原图，发信号前建议 `clone()`。
- 相机采图代码里就有 `emit sendImage(cvImg.clone(), workMode)` 这种做法。

## 4 本项目用到的设计模式

### 4.1 抽象工厂 / 工厂方法

相机、投影仪、机器人都用了工厂思想。

机器人抽象工厂：

```cpp
class AbstractRobotFactory : public QObject {
    Q_OBJECT
public slots:
    virtual std::shared_ptr<AbstractRobot> createRobot() = 0;
};
```

具体工厂：

```cpp
std::shared_ptr<AbstractRobot>
AnChuanRobotFactory::createRobot() {
    return std::make_shared<AnChuanRobot>();
}
```

好处：

- 上层不直接 `new AnChuanRobot`。
- 可以根据配置切换厂商。
- 降低业务层和设备厂商实现的耦合。

### 4.2 面向接口编程

上层持有的是抽象类：

```cpp
std::shared_ptr<AbstractRobot> robot;
std::shared_ptr<AbstractCameraFactory> cameraFactory;
std::shared_ptr<AbstractProjectorFactory> projectorFactory;
```

这表示上层关心“能力”，不关心具体品牌。

比如机器人只要能：

```cpp
connectRobot()
disconnectRobot()
welding()
moveL()
moveJ()
```

就能接入系统。

### 4.3 依赖注入

`RailWeldingSystem` 中：

```cpp
Rail *rail = nullptr;
WorkpieceCoarseLocalization *workpieceCoarseLocalization = nullptr;
```

这两个对象不是系统类内部直接创建，而是外部 UI 或模块创建后注入进来。

好处：

- 便于复用已有 UI 子模块。
- 降低模块创建顺序的耦合。
- 测试时可以替换成模拟对象。

### 4.4 外观模式 / 中介者模式

`RailWeldingSystem` 很像外观和中介者：

- 对 UI 来说，它提供统一入口。
- 对子模块来说，它负责协调流程。
- 它把“地轨完成、机器人完成、结构光完成、检测完成、轨迹规划完成”这些事件串起来。

新人改流程时，优先看 `RailWeldingSystem`，不要到每个子模块里乱连。

### 4.5 观察者模式

Qt 信号槽本质上就是观察者模式：

```text
发送者不关心谁接收
接收者订阅某个信号
信号发生时自动通知
```

例如：

```cpp
emit sendRobotMoveOver();
```

可能被系统流程、UI 状态灯、日志模块等多个地方监听。

### 4.6 策略模式

焊缝检测有不同策略：

- 点云检测：`SeamDetWithPointCloud`
- 分割检测：`SeamDetWithSeg`
- 不同焊缝区域：角接、对接、底板、横梁等

它们不是一个超大 `if else` 写到底，而是拆成不同类和算法模块。

### 4.7 状态机思想

焊接流程里有：

```cpp
WELD_MODE weldMode = MANUAL_WELD;
WELDING_STAGE weldingStage = PREPARE_STAGE;
```

自动焊接不是简单地“从头跑到尾”，而是根据状态推进：

```text
准备
粗定位
移动地轨和机器人
结构光扫描
焊缝检测
轨迹规划
焊接
下一个工件
```

新增流程时要先想清楚：

- 当前处于什么状态？
- 收到什么事件？
- 状态如何变化？
- 哪些标志位需要清零？

### 4.8 RAII 资源管理

RAII 的核心是“对象生命周期绑定资源生命周期”。

线程池就是例子：

```cpp
ThreadPool::~ThreadPool() {
    stopFlag = true;
    condition.notify_all();
    for (auto& t : workers) {
        if (t.joinable()) t.join();
    }
}
```

对象析构时自动停止线程并等待回收。

设备控制代码尤其需要 RAII 思维：

- 打开的相机要关闭。
- 连接的 PLC 要断开。
- CUDA 申请的内存要释放。
- 线程启动后要退出。

### 4.9 原型 / 深拷贝

`WeldSeamInfo` 有：

```cpp
std::shared_ptr<WeldSeamInfo> clone() const;
```

这是“复制复杂对象”的思路。因为焊缝对象里有点云、图像、端点、机器人位姿，如果只做浅拷贝，多个对象可能共享同一份内部数据，后面修改会互相影响。

新人要分清：

- 浅拷贝：只复制指针，底层数据共享。
- 深拷贝：底层数据也复制一份。

## 5 多线程和并行

### 5.1 为什么需要多线程

这个项目同时做很多事：

- UI 要保持响应。
- 相机要采图。
- 投影仪要触发。
- 点云要重建。
- 焊缝要检测。
- TensorRT 要推理。
- 地轨 PLC 要读取状态。
- 机器人要通讯和运动。

如果全部放在 UI 主线程，界面会卡死，设备响应也会变慢。

### 5.2 模块级线程

项目采用模块级线程：

```cpp
QThread *structLightCameraThread = new QThread;
QThread *seamDetWithPointCloudThread = new QThread;
QThread *seamDetWithSegThread = new QThread;
QThread *robotTrajectoryPlanningThread = new QThread;
QThread *errorSaveThread = new QThread;
QThread *robotThread = new QThread;
```

每个耗时模块放到自己的线程中，通过信号槽通讯。

### 5.3 计算任务线程池

`src/utils/common/ThreadPool` 是项目自定义线程池。

核心成员：

```cpp
std::vector<std::thread> workers;
std::queue<Task> tasks;
std::mutex queueMutex;
std::condition_variable condition;
std::atomic<bool> stopFlag;
```

工作方式：

```text
创建若干 worker 线程
        ↓
没有任务时 condition_variable 挂起等待
        ↓
addTask 加任务并唤醒一个线程
        ↓
worker 取出任务执行
        ↓
waitAll 等所有任务完成
```

### 5.4 future

`addTask` 返回：

```cpp
std::future<ReturnType>
```

`future` 表示“未来会有一个结果”。

典型使用：

```cpp
auto f = pool.addTask([] {
    return heavyCompute();
});

auto result = f.get();
```

注意：

- `get()` 会等待任务完成。
- 如果在 UI 线程里随便 `get()`，可能卡界面。

### 5.5 mutex 和 lock_guard

线程池保护任务队列：

```cpp
std::lock_guard<std::mutex> lock(queueMutex);
tasks.emplace(...);
```

意义：

- 多个线程同时访问队列会出问题。
- 加锁保证同一时刻只有一个线程修改队列。

Qt 里也有：

```cpp
QMutexLocker locker(&mutex_);
```

这是 Qt 风格的自动加锁，离开作用域自动解锁。

### 5.6 atomic

项目中有：

```cpp
std::atomic_bool forceStop = false;
std::atomic<size_t> unfinishedTasks{0};
```

`atomic` 用于简单的跨线程变量，不需要手动加锁。

适合：

- 停止标志。
- 任务数量。
- 简单状态位。

不适合：

- 复杂结构体。
- 多个变量要一起保持一致的情况。

### 5.7 OpenMP

工程开启了：

```text
QMAKE_LFLAGS += -openmp
QMAKE_CXXFLAGS += -openmp
```

`stable.h` 里也包含：

```cpp
#include <omp.h>
```

OpenMP 通常用于循环级并行，例如点云或矩阵计算。使用时要注意共享变量加锁或改为局部变量。

### 5.8 QtConcurrent / QFuture

项目包含：

```cpp
#include <QtConcurrent/QtConcurrent>
QFuture<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudFuture;
```

`QtConcurrent` 可以方便地把函数丢到线程池执行，`QFuture` 用来保存异步结果。

### 5.9 多线程常见坑

新同学尤其要避开这些问题：

- 在子线程直接操作 UI。
- 跨线程传自定义类型但没有 `qRegisterMetaType`。
- `cv::Mat` 发出去后原线程继续改同一块数据，导致显示错乱。
- 点云对象多个线程同时写。
- `QThread` 没有正确退出，程序关闭卡住。
- `QEventLoop` 等不到信号，流程卡死。
- 设备断开时，后台定时器还在读写。
- PCL 的 `pcl::ConcaveHull` 多线程同时运行可能不安全，项目里专门有 `QhullLock.h` 处理这类问题。

## 6 OpenCV 在项目里的用法

### 6.1 cv::Mat

`cv::Mat` 是 OpenCV 的图像矩阵类型。

项目里常见：

```cpp
cv::Mat img;
cv::Mat res;
std::vector<cv::Mat> images;
```

特点：

- 内部引用计数。
- 赋值通常不会复制像素，只是共享数据。
- 需要真正复制时用 `clone()`。

```cpp
emit sendImage(cvImg.clone(), workMode);
```

### 6.2 图像读写

```cpp
cv::Mat img = cv::imread("./data/icon/FY.png");
cv::imwrite("./data/error/origin.bmp", img);
```

调试算法时非常常用。

### 6.3 点、矩形和区域

项目常用：

```cpp
cv::Point2d
cv::Point3d
cv::Rect_<float>
cv::Rect_<double>
```

典型场景：

- 粗定位框。
- 焊缝区域框。
- 拍照视野范围。
- 图像坐标转世界坐标。

### 6.4 图像预处理

深度学习推理中常见：

```cpp
cv::resize(...)
cv::copyMakeBorder(...)
cv::dnn::blobFromImage(...)
```

作用：

- 缩放到模型输入大小。
- 补边保持比例。
- 转成神经网络输入格式。

### 6.5 后处理和绘制

YOLO 后处理会用：

```cpp
cv::dnn::NMSBoxes(...)
cv::rectangle(...)
cv::putText(...)
cv::addWeighted(...)
```

含义：

- `NMSBoxes` 去掉重叠检测框。
- `rectangle` 画框。
- `putText` 写类别和置信度。
- `addWeighted` 把分割 mask 半透明叠到原图上。

### 6.6 OpenCV 新人注意

- `cv::Mat` 为空时不要处理，先判断 `img.empty()`。
- OpenCV 默认 BGR，不是 RGB。
- `cv::Rect` 做 ROI 前要保证不越界。
- 多线程传图像时不确定是否会修改，就 `clone()`。
- 图像坐标是左上角为原点，世界坐标不是。

## 7 PCL 点云在项目里的用法

### 7.1 点云基本类型

项目最常用：

```cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
```

可以理解为：

```text
很多三维点组成的一片点云
每个点有 x, y, z
```

创建方式：

```cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
    new pcl::PointCloud<pcl::PointXYZ>);
```

或者：

```cpp
auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
    new pcl::PointCloud<pcl::PointXYZ>);
```

### 7.2 PCD 文件

保存点云：

```cpp
pcl::io::savePCDFile("pointCloud.pcd", *cloud);
```

读取点云：

```cpp
pcl::io::loadPCDFile("pointCloud.pcd", *cloud);
```

项目的 `data/common`、`data/error` 等目录里会出现 `.pcd` 文件。

### 7.3 滤波

`stable.h` 中包含了很多滤波器：

```cpp
pcl/filters/passthrough.h
pcl/filters/voxel_grid.h
pcl/filters/statistical_outlier_removal.h
pcl/filters/radius_outlier_removal.h
pcl/filters/extract_indices.h
```

常见用途：

- 直通滤波：按 x/y/z 范围裁剪点云。
- 体素滤波：降采样，减少点数量。
- 统计滤波：去掉离群点。
- 半径滤波：去掉孤立点。
- 索引提取：按算法结果提取某一部分点。

### 7.4 RANSAC 和模型拟合

项目中检测平面、直线等会用：

```cpp
pcl::SACSegmentation
pcl::SampleConsensusModelPlane
pcl::RandomSampleConsensus
pcl::ModelCoefficients
```

RANSAC 的思路：

```text
随机选一些点拟合模型
计算有多少点支持这个模型
重复很多次
选择支持点最多的模型
```

在焊缝项目里常用于：

- 拟合母材平面。
- 找焊缝所在平面。
- 提取边线。
- 判断端点。

### 7.5 KDTree 和邻域搜索

项目中包含：

```cpp
pcl::KdTreeFLANN
pcl::search::KdTree
```

作用：

- 快速找某个点附近的点。
- 聚类。
- 曲率分析。
- 端点判断。

### 7.6 聚类和区域增长

包含：

```cpp
pcl/segmentation/extract_clusters.h
pcl/segmentation/region_growing.h
```

用途：

- 把不同焊缝区域分开。
- 去掉小噪声块。
- 提取主要结构。

### 7.7 凸包 / 凹包

项目里包含：

```cpp
pcl/surface/concave_hull.h
```

`ConcaveHull` 用于求点云轮廓。README 里特别提醒多线程使用 `pcl::ConcaveHull` 要加锁，说明它可能不是线程安全的。

### 7.8 坐标变换

项目包含：

```cpp
pcl/common/transforms.h
Eigen::Matrix4f
```

典型任务：

```text
相机坐标系点云
        ↓ 标定矩阵
机器人坐标系点云
        ↓ 工艺偏移
机器人焊接位姿
```

新人一定要分清：

- 图像坐标。
- 相机坐标。
- 机器人坐标。
- 地轨/世界坐标。
- 工件局部坐标。

坐标系弄混，是这类项目最常见的大坑。

### 7.9 点云显示

UI 中使用：

```cpp
pcl::visualization::PCLVisualizer
QVTKOpenGLWidget / QVTKWidget
```

它把 PCL 点云嵌入 Qt 界面显示。

## 8 Eigen 和数学计算

项目里常见：

```cpp
Eigen::Vector3f
Eigen::Vector4f
Eigen::Matrix3f
Eigen::Matrix4f
```

用途：

- 坐标变换。
- 欧拉角提取。
- SVD 求解。
- 平面、直线、向量运算。
- 机器人姿态计算。

例子：

```cpp
Eigen::Vector3f center(
    (maxPt.x + minPt.x) / 2,
    (maxPt.y + minPt.y) / 2,
    (maxPt.z + minPt.z) / 2);
```

新人要掌握：

- 向量点乘、叉乘。
- 矩阵乘法顺序。
- 齐次坐标。
- 角度和弧度转换。
- 旋转矩阵和欧拉角不是一回事。

## 9 TensorRT / CUDA 深度学习推理

### 9.1 模块位置

```text
src/deepLearning/objectDetect
src/deepLearning/segment
```

项目里有：

- YOLO 目标检测。
- YOLO 分割。
- TensorRT engine 加载。
- CUDA 显存分配。
- OpenCV 前后处理。

### 9.2 TensorRT 核心对象

常见成员：

```cpp
nvinfer1::ICudaEngine *engine = nullptr;
nvinfer1::IRuntime *runtime = nullptr;
nvinfer1::IExecutionContext *context = nullptr;
cudaStream_t stream = nullptr;
```

理解：

- `runtime` 用来反序列化 engine。
- `engine` 是优化后的模型。
- `context` 是一次推理执行上下文。
- `cudaStream_t` 是 CUDA 异步执行流。

### 9.3 CUDA 内存

项目中会出现：

```cpp
cudaMallocAsync(...)
cudaHostAlloc(...)
cudaMemcpyAsync(...)
cudaStreamSynchronize(...)
cudaFree(...)
```

含义：

- GPU 侧内存要手动申请和释放。
- CPU 到 GPU、GPU 到 CPU 的拷贝是性能关键点。
- 异步拷贝后需要同步，才能安全读结果。

### 9.4 推理流程

典型深度学习流程：

```text
读取 cv::Mat 图像
        ↓
letterbox / resize / blobFromImage
        ↓
拷贝到 GPU
        ↓
TensorRT enqueue 推理
        ↓
拷贝输出到 CPU
        ↓
NMS / mask 后处理
        ↓
输出检测框、分割掩膜、可视化结果
```

### 9.5 新人注意

- `.engine` 和 CUDA/TensorRT 版本强相关，换环境可能不能用。
- 模型输入尺寸要和代码一致。
- BGR/RGB 通道顺序要确认。
- `cudaSetDevice(0)` 默认使用第 0 张显卡。
- TensorRT 指针释放顺序要谨慎。
- 推理线程和 UI 线程不要混在一起。

## 10 结构光相机模块

结构光模块主要由三部分组成：

```text
主相机
次相机
投影仪
```

对应对象：

```cpp
std::shared_ptr<AbstractCamera> primaryCamera;
std::shared_ptr<AbstractCamera> secondaryCamera;
std::shared_ptr<AbstractProjector> projector;
```

### 10.1 Basler 相机

项目使用 Basler Pylon SDK：

```cpp
#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>
```

抽象接口在：

```cpp
src/cameraFactory/AbstractCamera.h
```

具体实现在：

```cpp
src/cameraFactory/basler/BaslerCamera.*
```

相机常见操作：

- 枚举设备。
- 按序列号打开。
- 设置曝光、增益。
- 设置硬触发或软触发。
- 开始采图。
- 把图像转成 `cv::Mat`。
- 通过 `sendImage` 发给上层。

### 10.2 投影仪

抽象接口：

```cpp
class AbstractProjector : public QObject {
public slots:
    virtual bool open() = 0;
    virtual void close() = 0;
    virtual bool openLed() = 0;
    virtual bool closeLed() = 0;
    virtual bool triggerProj() = 0;
    virtual bool sendCmd(std::string cmd) = 0;
};
```

具体投影仪在：

```cpp
src/projectFactory/tengJu
```

项目通过 SDK 或命令控制投影仪：

- 打开设备。
- 设置亮度、帧率、投影图片数量。
- 打开 LED。
- 触发投影。
- 等待触发成功信号。

### 10.3 结构光采集流程

大致流程：

```text
连接主相机、次相机、投影仪
        ↓
设置相机触发模式和参数
        ↓
投影仪投图
        ↓
相机同步采图
        ↓
PointCloudReconstruction 重建点云
        ↓
输出工作台点云 / 焊缝区域点云
```

### 10.4 点云重建

位置：

```text
src/structLightCamera/reconstruction/PointCloudReconstruction.*
```

新人要从几个概念入手：

- 相机内参。
- 双目外参。
- 投影编码。
- 深度计算。
- 点云坐标生成。
- 点云裁剪和保存。

## 11 地轨 PLC 通讯

### 11.1 模块位置

```text
src/rail/Rail.h
src/rail/Rail.cpp
src/rail/RailWidget.*
```

### 11.2 libmodbus

项目使用：

```cpp
#include "modbus.h"
#include "modbus-tcp.h"
```

地轨连接：

```cpp
modbusTcp = modbus_new_tcp(ip.toStdString().c_str(), port);
modbus_connect(modbusTcp);
```

释放：

```cpp
modbus_close(modbusTcp);
modbus_free(modbusTcp);
```

### 11.3 线圈和寄存器

PLC 通讯里常见两个概念：

- 线圈：通常表示开关量，`true/false`。
- 寄存器：通常表示数值，例如速度、位置。

项目读写：

```cpp
modbus_write_bits(...)
modbus_write_registers(...)
modbus_read_bits(...)
modbus_read_registers(...)
```

### 11.4 float 和寄存器

PLC 寄存器通常是 16 位，`float` 是 32 位，所以需要两个寄存器拼起来。

项目中会看到：

```cpp
std::memcpy(&floatVal, &combined, sizeof(float));
```

新手要注意：

- 高低字顺序。
- 大端小端。
- PLC 地址从 0 还是从 1 开始。
- 写入速度、位置前要确认单位。

### 11.5 地轨控制流程

大致：

```text
连接 PLC
        ↓
使能 / 复位
        ↓
写速度和目标位置
        ↓
写运动命令线圈
        ↓
定时读当前位置和状态
        ↓
到位后发 sendAbsFinished
```

### 11.6 地轨新人注意

- 没连接 PLC 时禁止写线圈和寄存器。
- 通讯失败要能重连。
- 急停、停止、复位信号要谨慎。
- 自动流程里要等地轨和机器人都完成，再进入下一步。
- 实际设备调试要先低速、短距离。

## 12 机器人通讯和控制

### 12.1 抽象机器人接口

```cpp
class AbstractRobot : public QObject {
public slots:
    virtual bool connectRobot() = 0;
    virtual bool disconnectRobot() = 0;
    virtual bool welding() = 0;
    virtual bool moveL(robotPose p, double speed) = 0;
    virtual bool moveJ(robotJointAngle j, double speed) = 0;
};
```

机器人基本能力：

- 连接。
- 断开。
- 直线运动 `moveL`。
- 关节运动 `moveJ`。
- 焊接。
- 回传当前位姿和关节角。

### 12.2 机器人位姿

```cpp
class robotPose {
public:
    double x_ = 0;
    double y_ = 0;
    double z_ = 0;
    double a_ = 0;
    double b_ = 0;
    double c_ = 0;
};
```

通常表示：

- `x/y/z`：位置。
- `a/b/c`：姿态角。

关节角：

```cpp
class robotJointAngle {
public:
    double joint1 = 0;
    double joint2 = 0;
    double joint3 = 0;
    double joint4 = 0;
    double joint5 = 0;
    double joint6 = 0;
};
```

### 12.3 安川和宝元

项目中有不同机器人实现：

```text
src/robotFactory/an_chaun
src/robotFactory/bao_yuan
```

它们都继承 `AbstractRobot`，但底层通讯方式不同：

- 安川可能通过 TCP/socket、机器人端脚本或队列命令通讯。
- 宝元使用 `BaoYuanRobot` SDK，依赖 `sc2_vc_x64.lib/dll`。

上层不应该关心细节，只调用抽象接口。

### 12.4 机器人控制常见坑

- 单位：mm 还是 m，角度还是弧度。
- 坐标系：机器人基坐标、工具坐标、相机坐标。
- 姿态角顺序：ABC、RPY、ZYX 等不要混。
- 速度倍率要确认。
- 自动焊接前要确认轨迹无碰撞。
- 焊接中断要有 `forceStop` 这种安全标志。

## 13 三端通讯怎么理解

这里的“三端”可以按实际系统理解为：

```text
上位机 PC 软件
        ↙          ↘
机器人控制器      PLC / 地轨
        ↘          ↙
       相机/投影仪/传感器辅助感知
```

更具体地说：

- PC 端：Qt 程序，负责 UI、算法、流程调度。
- 机器人端：接收运动、焊接、查询位姿等命令。
- PLC/地轨端：接收地轨运动、复位、停止、速度位置等命令。
- 相机/投影仪端：负责采图和结构光扫描，是感知设备端。

### 13.1 PC 和 UI 内部通讯

使用 Qt 信号槽：

```cpp
emit sendAutoWelding();
connect(...);
```

特点：

- 类型安全。
- 支持跨线程。
- 适合模块解耦。

### 13.2 PC 和 PLC 通讯

使用 modbus TCP：

```cpp
modbus_new_tcp(ip, port)
modbus_read_registers
modbus_write_registers
```

特点：

- 面向寄存器。
- 指令简单。
- 要自己管理地址和数据解析。

### 13.3 PC 和机器人通讯

根据机器人品牌不同：

- SDK 调用。
- TCP socket。
- 机器人端脚本。
- 文件路点或命令队列。

上层统一成：

```cpp
moveL(...)
moveJ(...)
welding()
```

### 13.4 PC 和相机/投影仪通讯

相机：

```text
Basler Pylon SDK
```

投影仪：

```text
TJSTProjectorApi / sendCmd
```

### 13.5 自动流程的同步

典型同步逻辑：

```cpp
if (railAbsActionFinishedFlag && robotMoveLFinishedFlag) {
    emit sendUpdataWorkbench();
}
```

这表示：

```text
地轨到位
机器人到位
        ↓
才能开始结构光扫描
```

自动化项目最重要的是“等正确的设备完成，再做下一步”。

## 14 焊缝数据结构

核心数据类：

```cpp
class WeldSeamInfo {
public:
    int areaNum = -1;
    cv::Mat originalImg;
    cv::Mat weldAreaImg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr weldAreaPointCloudInCamera;
    pcl::PointCloud<pcl::PointXYZ>::Ptr weldAreaPointCloudInRobot;
    std::shared_ptr<std::vector<pcl::PointXYZ>> weldEndPointsInCamera;
    std::shared_ptr<std::vector<pcl::PointXYZ>> weldEndPointsInRobot;
    std::vector<robotPose> robotWeldPose;
};
```

它把一条焊缝相关的所有信息放在一起：

- 原始图像。
- 焊缝区域图像。
- 分割结果。
- 相机坐标系点云。
- 机器人坐标系点云。
- 焊缝端点。
- 焊缝类型。
- 机器人焊接位姿。
- 碰撞检测结果。

新人做算法时，很多结果最后都要写进 `WeldSeamInfo`。

## 15 焊缝检测流程

### 15.1 点云方法

位置：

```text
src/seamDetWithPointCloud
```

常见步骤：

```text
输入焊缝区域点云
        ↓
滤波、裁剪、去噪
        ↓
拟合平面或线
        ↓
提取焊缝候选区域
        ↓
求端点
        ↓
写入 WeldSeamInfo
```

不同焊缝区域有不同类：

```text
BeamButtSeamsDet
CornerButtSeamsDet
UpBeamFilletSeamsDet
DownBeamFilletSeamsDet
BottomPlateFilletSeamsDet
```

### 15.2 分割方法

位置：

```text
src/seamDetWithSeg
```

大致：

```text
输入图像
        ↓
YOLO 分割得到 mask
        ↓
结合点云坐标
        ↓
从 mask 区域反查三维点
        ↓
求焊缝端点和区域
```

### 15.3 目标检测和分割结合

粗定位和焊缝检测里既有检测框，也有分割 mask：

- 检测框用于快速定位区域。
- 分割 mask 用于精细边界。
- 点云用于三维坐标和机器人轨迹。

## 16 轨迹规划

位置：

```text
src/robotTrajectoryPlanning
```

轨迹规划的输入通常是：

- 焊缝端点。
- 焊缝类型。
- 母材平面。
- 安全偏移。
- 当前机器人姿态。
- 工艺参数。

输出通常是：

- `std::vector<robotPose>` 焊接位姿序列。
- 摆焊参考点。
- 碰撞检测结果。

新人要注意：

- 轨迹点顺序非常重要。
- 起弧、收弧、过渡点要分清。
- 姿态平滑比单点正确更重要。
- 规划完成后通过信号触发焊接。

## 17 配置、日志和崩溃排查

### 17.1 配置

项目配置主要在：

```text
data/config/*.json
```

使用 `cereal` 读写：

```cpp
cereal::JSONInputArchive archive(is);
archive(cereal::make_nvp("Cameras", cameraConfigMap));
```

配置常包含：

- 结构光参数。
- 相机内外参。
- 轨迹规划参数。
- 粗定位标定参数。
- 地轨方向。

### 17.2 日志

使用 `plog`：

```cpp
plog::init(plog::debug, "./data/log/log.csv", 1000000000, 100);
```

建议：

- 设备连接成功/失败要打日志。
- 自动流程阶段切换要打日志。
- 关键坐标和轨迹数量要打日志。
- 异常返回要写清楚原因。

### 17.3 崩溃转储

```cpp
CrashHandler::Init(L"data/debug");
```

崩溃后会在 `data/debug` 下生成文件，便于定位。

## 18 构建系统和宏

### 18.1 qmake

主工程：

```text
ground_rail_welding.pro
```

开启模块：

```text
QT += core gui serialbus network opengl
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
CONFIG += c++11
```

### 18.2 预编译头

```text
PRECOMPILED_HEADER = src/stable.h
```

`stable.h` 放了大量第三方库头文件，可以减少编译时间。

### 18.3 宏定义

项目里有：

```text
SMART_CAMERA
LI_CONFIG
A17_CONFIG
ROM_CONFIG
GONG_RAIL_CONFIG
```

宏用于选择编译配置、第三方库路径和功能开关。

新手不要随便改宏，改之前先确认：

- 对应的 `.pri` 是否存在。
- 第三方库路径是否正确。
- 模型文件是否匹配。
- DLL 是否放到运行目录。

## 19 UI 层怎么读

主界面：

```text
src/ui/RailWeldingMainWindow.*
```

UI 层主要做：

- 初始化控件。
- 连接按钮和槽函数。
- 显示图片和点云。
- 显示设备状态灯。
- 显示焊缝表格和日志。
- 把用户操作变成信号发给系统。

UI 层不应该做复杂算法。

例如按钮点击：

```cpp
void RailWeldingMainWindow::on_btnConnectStructLight_clicked() {
    emit connectStructLightCamera();
}
```

这才是推荐风格：UI 只发命令，业务逻辑交给系统或模块。

## 20 新增功能时的标准路线

假设你要新增一个“保存当前点云和图像”的功能，不要一上来乱改。

推荐路线：

1. 找数据在哪里产生：结构光模块还是焊缝检测模块。
2. 找谁需要触发：按钮、自动流程、调试命令。
3. 如果是 UI 触发，在 UI 里发一个 signal。
4. 在业务模块里新增 slot。
5. 在 `connect` 里把 signal 和 slot 连起来。
6. 在 slot 里做保存逻辑。
7. 用 `plog` 打日志。
8. 保存失败要通过信号告诉 UI。
9. 多线程数据要复制或加锁。

新增设备品牌时：

1. 新建具体设备类，继承抽象接口。
2. 实现所有纯虚函数。
3. 新建具体工厂类。
4. 在系统初始化中选择新工厂。
5. 保持上层调用接口不变。

新增焊缝算法时：

1. 确认输入是图像、点云还是二者结合。
2. 新建算法类或继承现有抽象检测类。
3. 输出统一写入 `WeldSeamInfo`。
4. 不要破坏现有焊缝类型枚举。
5. 添加日志和错误数据保存。
6. 使用离线 `.pcd` 和图片先测试，再上设备。

## 21 常见排错清单

### 21.1 程序启动失败

检查：

- Qt 运行环境。
- 第三方 DLL 是否在 `release/debug` 目录。
- PCL/VTK/OpenCV/TensorRT 路径是否匹配。
- `.engine` 文件是否存在。
- `data/config` 是否缺文件。

### 21.2 相机连接失败

检查：

- Basler Pylon Viewer 是否能看到相机。
- 相机是否被其他程序占用。
- IP 和网卡是否同网段。
- 序列号配置是否正确。
- 触发模式是否正确。

### 21.3 投影仪不触发

检查：

- SDK 是否加载。
- 投影仪是否连接。
- `fps/projNum/brightness` 参数是否正确。
- 是否等待了 `triggerProjSuccess`。

### 21.4 PLC 通讯失败

检查：

- IP 和端口。
- PLC 是否运行。
- modbus TCP 是否开启。
- 寄存器地址是否正确。
- 是否被防火墙拦截。
- 写入数据格式是否正确。

### 21.5 机器人不动

检查：

- 机器人是否上电、使能。
- 安全门、急停、报警状态。
- 坐标和速度是否超限。
- 通讯是否连接。
- 机器人端程序是否启动。
- 防火墙是否关闭

### 21.6 点云为空

检查：

- 相机是否采到图。
- 投影是否触发。
- 标定文件是否加载。
- 重建 ROI 是否过窄。
- 点云被滤波条件全部过滤。

### 21.7 焊缝检测失败

检查：

- 输入图像是否正常。
- 分割 mask 是否正确。
- 点云坐标是否和图像对齐。
- 焊缝区域类型是否判断正确。
- RANSAC 阈值是否过严。
- 端点求解是否受噪声影响。

### 21.8 自动流程卡住

检查：

- 哪个信号没有发出。
- 哪个 `connect` 没连上。
- 哪个线程没有事件循环。
- `railAbsActionFinishedFlag` 和 `robotMoveLFinishedFlag` 是否正确清零和置位。
- 是否有 `QEventLoop` 一直等不到退出信号。

## 22 新人学习顺序

建议按这个顺序学，不要一开始就冲进 深度学习 或点云细节：

1. C++ 类、指针、引用、智能指针、容器。
2. Qt 的 `QObject`、信号槽、`QThread`。
3. 读 `main.cpp` 和 `RailWeldingMainWindow`。
4. 读 `RailWeldingSystem`，画出自动流程图。
5. 读 `AbstractCamera / AbstractRobot / AbstractProjector`，理解接口和多态。
6. 读 `Rail`，理解 modbus 设备通讯。
7. 读 `StructLightCamera`，理解相机、投影仪、点云输出。
8. 读 `WeldSeamInfo`，理解焊缝数据结构。
9. 读 `SeamDetWithPointCloud` 和点云工具函数。
10. 读 `SeamDetWithSeg` 和深度学习推理。
11. 读 `RobotTrajectoryPlanning`，理解焊缝结果如何变成机器人轨迹。
12. 最后再看 CUDA/TensorRT 细节和性能优化。

## 23 最重要的工程原则

这几个原则比背语法更重要：

- UI 只做显示和交互，不写重算法。
- 模块间通过信号槽通信，不随便互相调用内部细节。
- 设备接口用抽象类隔离厂商实现。
- 跨线程传大数据要考虑拷贝、生命周期和注册元类型。
- 点云、图像、机器人坐标必须明确坐标系。
- 自动流程要用状态和完成信号推进，不能靠猜时间。
- 设备控制先保证安全，再追求自动化。
- 新算法先离线跑通，再接真实设备。
- 每个可能失败的设备操作都要有日志和 UI 提示。
- 不懂某段逻辑时，先画数据流和信号流，不要急着改代码。

## 24 一句话总图

地轨项目的本质是：

```text
Qt UI 接收人的操作
        ↓
RailWeldingSystem 调度流程
        ↓
相机和投影仪采集三维点云
        ↓
OpenCV / TensorRT / PCL 检测焊缝
        ↓
Eigen / 轨迹规划生成机器人位姿
        ↓
PLC 控制地轨，机器人控制器执行焊接
        ↓
日志、错误数据和 UI 状态持续反馈
```

学这个项目，不是只学 C++ 语法，而是学会把 C++、Qt、多线程、图像、点云、深度学习和工业设备通讯组合成一个可靠的自动化系统。

## 25 3EA 项目深入补充篇

前面的内容已经把 C++、Qt、PCL、结构光、机器人和地轨做了总览。本节再按 3EA-Welding 的真实代码，把“数据从哪里来、经过谁、最后变成什么”讲细一点。

读这个项目时，建议始终抓住三条主线：

```text
控制流：
UI 按钮 / 自动流程
        ↓
RailWeldingSystem
        ↓
结构光 / 焊缝检测 / 轨迹规划 / 机器人 / 三轴

数据流：
cv::Mat 图像
        ↓
结构光重建 pcl::PointCloud
        ↓
WeldSeamInfo
        ↓
robotPose / SeamCoordinate.txt

线程流：
主 UI 线程
        ↓
各模块 QObject moveToThread
        ↓
模块内部 ThreadPool / QtConcurrent / OpenMP
```

### 25.1 入口和程序启动顺序

入口文件：

```text
main.cpp
```

启动时做了几件关键事：

```cpp
QApplication a(argc, argv);
vtkOutputWindow::SetGlobalWarningDisplay(0);
CrashHandler::Init(L"data/debug");
initPlog();
registerMetaType();

WeldingMainWindow w;
w.show();
return a.exec();
```

这里要注意：

- `QApplication` 创建以后，Qt 的事件循环才有意义。
- `CrashHandler::Init(L"data/debug")` 用来生成崩溃转储。
- `plog::init(...)` 把日志写到 `./data/log/log.csv`。
- `registerMetaType()` 是跨线程信号槽的基础。

项目里注册了很多复杂类型：

```cpp
qRegisterMetaType<cv::Mat>("cv::Mat");
qRegisterMetaType<robotPose>("robotPose");
qRegisterMetaType<robotJointAngle>("robotJointAngle");
qRegisterMetaType<pcl::PointCloud<pcl::PointXYZ>::Ptr>(
    "pcl::PointCloud<pcl::PointXYZ>::Ptr");
qRegisterMetaType<std::vector<std::shared_ptr<WeldSeamInfo>>>(
    "std::vector<std::shared_ptr<WeldSeamInfo>>");
```

原因是：这些对象会通过信号槽在线程之间传递。如果没有注册，直接连接跨线程信号时可能运行时警告，甚至信号传不过去。

### 25.2 总调度类 RailWeldingSystem

核心调度位置：

```text
src/weldingSystem/RailWeldingSystem.h
src/weldingSystem/RailWeldingSystem.cpp
```

这个类不是算法类，而是系统中枢。它负责：

- 创建结构光相机模块。
- 创建点云焊缝检测模块。
- 创建分割焊缝检测模块。
- 创建轨迹规划模块。
- 创建机器人对象。
- 接收粗定位结果。
- 推进自动焊接流程。

构造函数里的初始化顺序非常重要：

```cpp
initStructLightCamera();
initSeamDetWithPointCloud();
initSeamDetWithSeg();
initTrajectoryPlanning();
initRobot();
```

对应的数据链路是：

```text
StructLightCamera::sendWeldAreaInfoGF
        ↓
SeamDetWithPointCloud::whenDetSeamWithPointCloudGF
        ↓
SeamDetWithPointCloud::sendDetSeamWithPointCloud
        ↓
SeamDetWithSeg::whenDetSeamWithSeg
        ↓
SeamDetWithSeg::sendDetSeamWithSeg
        ↓
AbstractTrajectoryPlanning::whenPlanningTrajectory
        ↓
AbstractTrajectoryPlanning::sendPlannedSeams
        ↓
RailWeldingSystem::whenGetFinalSeams
```

也就是说，`RailWeldingSystem` 自己不处理点云、不做分割、不算机器人姿态，它只负责把模块接起来。

### 25.3 本项目最核心的数据结构 WeldSeamInfo

位置：

```text
src/utils/common/WeldSeamInfo.h
```

`WeldSeamInfo` 是焊缝生命周期的“档案袋”。一条焊缝从图像到点云、从检测到规划，相关数据都逐步写进这个对象。

重要字段可以按阶段理解。

焊缝区域阶段：

```cpp
int areaNum;
cv::Mat originalImg;
cv::Mat weldAreaImg;
std::shared_ptr<cv::Rect_<float>> rectPtr;
pcl::PointCloud<pcl::PointXYZ>::Ptr weldAreaPointCloudInCamera;
pcl::PointCloud<pcl::PointXYZ>::Ptr weldAreaPointCloudInRobot;
WELD_AREA_TYPE weldAreaType;
```

含义：

- `originalImg` 是原始图像。
- `weldAreaImg` 是目标检测框或分割框裁出来的焊缝区域图。
- `rectPtr` 记录这个区域在原图中的位置。
- `weldAreaPointCloudInCamera` 是相机坐标系下的局部点云。
- `weldAreaPointCloudInRobot` 是机器人坐标系下的局部点云。
- `weldAreaType` 表示区域类型，例如板板、管板、管管等。

焊缝检测阶段：

```cpp
bool detectSuccFlag;
std::shared_ptr<std::vector<pcl::PointXYZ>> weldEndPointsInCamera;
std::shared_ptr<std::vector<pcl::PointXYZ>> weldEndPointsInRobot;
pcl::ModelCoefficients::Ptr weldCoeff;
std::vector<pcl::ModelCoefficients::Ptr> otherSurface;
WELD_TYPE weldType;
pcl::ModelCoefficients::Ptr seamsLineToVal;
```

含义：

- `detectSuccFlag` 是最终是否检测成功。
- `weldEndPointsInCamera` 是相机坐标系下焊缝端点或曲线点。
- `weldEndPointsInRobot` 是转换到机器人坐标系后的结果。
- `weldCoeff` 常用于保存焊缝所在平面。
- `otherSurface` 保存相邻母材平面、圆柱等几何模型。
- `seamsLineToVal` 是用来验证结果的参考直线。

分割修正阶段：

```cpp
std::shared_ptr<std::vector<pcl::PointXYZ>> weldEndPointsFromSeg;
cv::Mat segResultImg;
cv::Mat segMaskImg;
double width = 1.5;
```

含义：

- `weldEndPointsFromSeg` 是分割方法反算出的三维焊缝点。
- `segMaskImg` 是实例分割 mask。
- `width` 是根据分割区域估计出的焊缝宽度。

轨迹规划阶段：

```cpp
std::vector<robotPose> robotWeldPose;
std::vector<double> swingReferencePoints;
std::vector<CollisionResult> weldCollisionResult;
```

含义：

- `robotWeldPose` 是规划后的机器人焊接位姿。
- `swingReferencePoints` 是摆焊参考点。
- `weldCollisionResult` 是碰撞检测和避让补偿结果。

读代码时，不要把 `WeldSeamInfo` 看成普通结构体。它其实是整个系统的数据合同。

### 25.4 工件和焊缝类型枚举

项目里有两套类型要分清：

```cpp
enum WELD_AREA_TYPE
enum WELD_TYPE
```

`WELD_AREA_TYPE` 更像“检测框区域是什么”，例如：

```cpp
Plate_Plate_F
TubeSide_Plate_F
Tube_Plate_F
Tube_Tube_F
```

`WELD_TYPE` 更像“最终要怎么焊”，例如：

```cpp
Plate_Plate_Fillet_V
Plate_Plate_Fillet_H
TubeSide_Plate_F_H
Tube_Plate_Fillet
Tube_Tube_Fillet
```

一个区域可能在点云算法里继续拆成多条焊缝，所以区域类型和焊缝类型不是一回事。

### 25.5 结构光模块怎么读

核心位置：

```text
src/structLightCamera/StructLightCamera.h
src/structLightCamera/StructLightCamera.cpp
src/structLightCamera/reconstruction/PointCloudReconstruction.h
src/structLightCamera/reconstruction/PointCloudReconstruction.cpp
```

`StructLightCamera` 管硬件和采集：

- 主相机 `primaryCamera`。
- 次相机 `secondaryCamera`。
- 投影仪 `projector`。
- 点云重建器 `PointCloudReconstruction`。

结构光模块内部也用了多线程：

```cpp
QThread *primaryCameraThread = new QThread;
QThread *secondaryCameraThread = new QThread;
QThread *projectorThread = new QThread;
```

普通重建和工件重建要区分：

```cpp
enum RECONSTRUCTION_MODE {
    COMMON,
    WORKPIECE
};
```

普通重建更偏调试，看整个局部点云。

工件模式会走焊缝区域检测、区域 mask、局部点云重建，并输出 `WeldSeamInfo`。

### 25.6 结构光点云重建算法链路

`PointCloudReconstruction` 的核心流程可以按注释编号读：

```text
imageDistribute
        ↓
makeMaskForReconstruct / makeMaskForSeamsDetToGF
        ↓
solveWrapPhase
        ↓
decodeGrayCode
        ↓
phaseUnwrap
        ↓
cameraProjectMatch
        ↓
calcPointCloud
        ↓
pointCloudPostProcess
```

对应意义：

1. `imageDistribute()`  
   把采集图像拆成相移图和格雷码图。

2. `solveWrapPhase()`  
   通过多步相移求每个像素的包裹相位。

3. `decodeGrayCode()`  
   通过格雷码判断像素所在投影周期。

4. `phaseUnwrap()`  
   结合包裹相位和格雷码，得到绝对相位。

5. `cameraProjectMatch()`  
   把相机像素点和投影仪坐标匹配起来。

6. `calcPointCloud()`  
   用相机投影矩阵、投影仪投影矩阵求三维点。

7. `pointCloudPostProcess()`  
   做坐标变换、缩放、滤波等后处理。

这里最难的是 `calcPointCloud()`。它本质是在求：

```text
相机射线
投影仪光平面 / 投影仪约束
        ↓
三维空间交点
```

项目中还用 OpenMP 加速逐点计算：

```cpp
int maxThreads = omp_get_max_threads();
```

新人读这一段时，不必一开始就完全推导公式。先记住输入输出：

```text
输入：cameraCoord + projectCoord + 标定参数
输出：pcl::PointCloud<pcl::PointXYZ>
```

### 25.7 工件模式下的区域点云重建

龙门支架模式主要看：

```cpp
std::vector<std::shared_ptr<WeldSeamInfo>>
PointCloudReconstruction::weldAreaReconstructToGF()
```

它大致做：

```text
初始化上下文
        ↓
图像分发
        ↓
根据目标检测 / 实例分割生成 mask
        ↓
相移 + 格雷码解码
        ↓
重建工作台背景平面
        ↓
重建每个焊缝区域点云
        ↓
写入 WeldSeamInfo
```

相关函数：

```cpp
makeMaskForSeamsDetToGF();
reconstructForWorkbench();
reconstructForSeamArea();
```

`makeMaskForSeamsDetToGF()` 里使用实例分割结果，生成每个焊缝区域对应的 mask 和 `WeldSeamInfo`。

`reconstructForWorkbench()` 会拟合背景平面：

```cpp
pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr modelPlane(...);
pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(modelPlane);
```

背景平面拟合出来后，会把离背景平面太近的点去掉，保留工件和焊缝相关点。

`reconstructForSeamArea()` 会把每个局部区域的重建点云写入：

```cpp
areaInfo->weldAreaPointCloudInCamera = reconstructPointCloud;
```

所以：如果后面的焊缝检测点云为空，优先回头检查 `makeMaskForSeamsDetToGF()` 和 `reconstructForSeamArea()`。

### 25.8 点云焊缝检测总入口

位置：

```text
src/seamDetWithPointCloud/SeamDetWithPointCloud.h
src/seamDetWithPointCloud/SeamDetWithPointCloud.cpp
```

角钢和龙门支架分别有入口：

```cpp
whenDetSeamWithPointCloud(...)
whenDetSeamWithPointCloudGF(...)
```

龙门支架的分发关系：

```cpp
gantrayFrameSeamsDet[WELD_AREA_TYPE::Plate_Plate_F]
    = []() { return std::make_shared<PlatePlateFilletSeamsDet>(nullptr); };

gantrayFrameSeamsDet[WELD_AREA_TYPE::TubeSide_Plate_F]
    = []() { return std::make_shared<TubeSidePlateFilletSeamsDet>(nullptr); };

gantrayFrameSeamsDet[WELD_AREA_TYPE::Tube_Plate_F]
    = []() { return std::make_shared<TubePlateFilletSeamsDet>(nullptr); };

gantrayFrameSeamsDet[WELD_AREA_TYPE::Tube_Tube_F]
    = []() { return std::make_shared<TubeTubeFilletSeamsDet>(nullptr); };
```

这里体现了策略模式：同一个入口，根据 `weldAreaType` 创建不同算法类。

每个具体算法都继承：

```cpp
class AbstractSeamDet : public QObject {
public:
    virtual std::vector<std::shared_ptr<WeldSeamInfo>>
    solveSeamsEndPoints(std::vector<std::shared_ptr<WeldSeamInfo>> seamsInfo) = 0;
};
```

也就是说，每个点云算法类只需要完成一件事：

```text
输入若干 WeldSeamInfo
        ↓
读取 weldAreaPointCloudInCamera
        ↓
拟合平面 / 圆柱 / 直线 / 边界
        ↓
写入 weldEndPointsInCamera、weldCoeff、otherSurface、weldType
```

### 25.9 龙门支架点云算法怎么分类

龙门支架检测类主要在：

```text
src/seamDetWithPointCloud/gantrayFrameDet
```

常见几类：

```text
platePlateFilletSeamsDet
tubeSidePlateFilletSeamsDet
tubePlateFilletSeamsDet
tubeTubeFilletSeamsDet
```

可以这样理解：

- 板板角接：主要找两个平面交线或角接位置。
- 管侧板角接：圆柱侧面和板/肋板之间的角接。
- 管板角接：圆柱和板平面的交线或贴近交线区域。
- 管管角接：两个圆柱相交或接近处的曲线焊缝。

点云算法常用工具：

```cpp
pcl::SACSegmentation
pcl::RandomSampleConsensus
pcl::ProjectInliers
pcl::EuclideanClusterExtraction
pcl::KdTreeFLANN
pcl::ConcaveHull
Eigen::Vector3f
Eigen::Matrix4f
```

项目 README 特别提醒：如果多个线程同时使用 `pcl::ConcaveHull`，要加锁。这个项目里有：

```text
src/seamDetWithPointCloud/QhullLock.h
```

原因是 Qhull 相关实现不是线程安全的。

### 25.10 分割方法不是替代点云，而是修正点云（三轴没有使用）

位置：

```text
src/seamDetWithSeg/SeamDetWithSeg.h
src/seamDetWithSeg/SeamDetWithSeg.cpp
```

分割方法处理流程：

```text
接收点云方法输出的 WeldSeamInfo
        ↓
对部分对接焊缝做 YOLO 实例分割
        ↓
mask 白色像素提取
        ↓
像素点根据焊缝平面反算三维点
        ↓
RANSAC 拟合三维直线
        ↓
得到分割端点和焊缝宽度
        ↓
与点云方法结果融合
```

核心反投影函数：

```cpp
bool point2dTo3d(std::vector<cv::Point>& pixelSet,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr& seamPointsSet,
                 Plane& p);
```

这里使用的是平面约束。二维像素点本身不能唯一确定三维点，只能确定一条相机射线。加入焊缝所在平面后，射线和平面相交，才能得到三维坐标。

融合逻辑在：

```cpp
fusionPointCloudAndSegRes(...)
```

它不是无脑采用分割结果，而是用距离阈值验证：

- 点云端点到参考直线距离是否合理。
- 分割端点到点云结果距离是否合理。
- 如果点云失败但分割通过验证，才让分割接管。
- 如果两者都不可信，将 `detectSuccFlag` 置为 `false`。

这也是工业项目里常见思路：深度学习给候选，几何算法给约束，最终用规则融合。

### 25.11 粗定位和拍照规划

位置：

```text
src/workpieceCoarseLocalization
src/photoPlanner
```

粗定位模块输出：

```cpp
std::shared_ptr<workpieceBoxInWorld>
```

对应结构在：

```text
src/workpieceCoarseLocalization/include/maskImageProcessConfig.hpp
```

其中重要字段：

```cpp
std::vector<cv::Rect_<double>> weldAreaRect;
std::vector<cv::Point3d> photoPos;
std::vector<std::vector<cv::Rect_<double>>> rectOfPhotoPos;
```

粗定位不是直接焊接，它主要解决三个问题：

1. 工件在哪。
2. 每个工件有哪些焊缝区域。
3. 结构光应该到哪些拍照位置采集。

`RailWeldingSystem::whenGetCoarseLocalization()` 会接收粗定位结果，然后调用 `PhotoPlanner` 规划拍照位置。

自动流程里，移动到某个工件时会做：

```text
选择表格行
        ↓
取该工件下一个 photoPos
        ↓
计算机器人拍照位姿
        ↓
保存当前拍照覆盖的 rectOfPhotoPos
        ↓
机器人移动
        ↓
机器人和地轨都到位后扫描
```

如果后续精扫漏焊缝，要检查：

- 粗定位 `weldAreaRect` 是否正确。
- `PhotoPlanner` 是否覆盖该区域。
- `rectOfPhotoPos` 是否传到结构光端。
- 拍照偏移 `photoPosOffsetX/Y/Z` 是否适合现场。

### 25.12 轨迹规划怎么把点云变成 robotPose

位置：

```text
src/robotTrajectoryPlanning
```

统一接口：

```cpp
class AbstractTrajectoryPlanning : public QObject {
public slots:
    virtual void whenPlanningTrajectory(
        std::vector<std::shared_ptr<WeldSeamInfo>> weldSeamInfo) = 0;
};
```

两个主要实现：

```text
SteelAngleTrajectoryPlanning
GantrayFrameTrajectoryPlanning
```

角钢轨迹规划更偏直线焊缝和左右侧补偿。龙门支架轨迹规划更复杂，包含曲线、圆柱、碰撞检测和摆焊参考点。

龙门支架规划主流程可以按函数名读：

```text
normalizeSurfaceDirectionInCamera
        ↓
transSeams2Base
        ↓
determineWorkpieceOri
        ↓
transSeamsOri
        ↓
compensateSeams
        ↓
generateWeldPose
        ↓
debugWeldingCollisionCheck
        ↓
computeSwingReferencePointsForSeam
        ↓
sendPlannedSeams
        ↓
sendTrajectoryPlanOver
```

重点理解坐标变换：

```cpp
const Eigen::Matrix4f& EyH = trajectoryConfig.matrixEyeHand;
const Eigen::Matrix4f& EnB = trajectoryConfig.matrixEnd2Base;
```

常见组合：

```text
相机坐标系点
        ↓ matrixEyeHand / matrixEnd2Base
机器人基坐标系点
        ↓ 焊枪姿态规划
robotPose
```

`robotPose` 字段：

```cpp
double x_, y_, z_;
double a_, b_, c_;
```

前 3 个是位置，后 3 个是姿态角。项目里尤其要注意：

- `a/b/c` 的旋转顺序。
- 角度制和弧度制。
- 机器人品牌对姿态定义可能不同。
- 同一焊缝在机器人左侧、右侧、前方时，方向要统一处理。

### 25.13 焊接路点文件怎么生成

轨迹规划完成后，`RailWeldingSystem::whenGetFinalSeams()` 会调用：

```cpp
robotTrajectoryPlanning->write2File(
    this->weldAreaInfo,
    this->robotTrajectoryPlanning->endOfLeftSeamSerial);
```

写出的文件：

```text
./data/SeamCoordinate.txt
```

底层写点函数：

```cpp
writeWeldPoint(...)
```

一个完整焊缝通常不是只写两个点，而是：

```text
安全过渡点
        ↓
接近起弧点
        ↓
起弧焊接点
        ↓
中间焊接点或曲线采样点
        ↓
收弧点
        ↓
退出过渡点
```

所以你在 `robotWeldPose` 里看到两个端点，最后写入文件时可能会扩展出更多机器人运动点。

### 25.14 碰撞检测和摆焊参考点

龙门支架里有碰撞检查：

```text
src/robotTrajectoryPlanning/utils/debugcollisioncheck.h
src/robotTrajectoryPlanning/utils/debugcollisioncheck.cpp
```

调用入口：

```cpp
debugWeldingCollisionCheck(weldSeamInfo);
```

结果写入：

```cpp
info->weldCollisionResult
```

对于管板、管管等焊缝，焊枪靠近圆柱时可能干涉。项目会估计一个额外退让量：

```cpp
extraOffset
```

然后在写路点时通过：

```cpp
applyWeldGunWithdraw(...)
```

让焊枪沿姿态方向适当后撤。

摆焊参考点计算入口：

```cpp
computeSwingReferencePointsForSeam(...)
```

不同焊缝调用不同函数：

```cpp
computePlatePlateFilletVerticalSwingPoints(...)
computeTubePlateFilletSwingPoints(...)
computeTubeTubeFilletSwingPoints(...)
```

这些函数输出的 `swingReferencePoints` 本质上是给机器人端摆焊动作提供参考方向或参考点。

### 25.15 三轴地轨和 PLC 通讯细化

位置：

```text
src/rail
src/rail/concrete_axis
```

三轴枚举：

```cpp
enum class Axis : uint8_t {
    X = 0,
    Y = 1,
    Z = 2,
    ALL = 3
};
```

每个轴的地址通过偏移计算：

```cpp
constexpr int AXIS_B_OFFSET = 0x000B;
constexpr int AXIS_R_OFFSET = 0x0018;
constexpr int AXIS_S_OFFSET = 0x0020;
constexpr int AXIS_D_OFFSET = 0x000A;

constexpr int addr(Axis axis, RegB reg) {
    return static_cast<int>(reg) + axisIndex(axis) * AXIS_B_OFFSET;
}
```

这意味着：X、Y、Z 三个轴不是写三套重复代码，而是同一套寄存器定义加轴号偏移。

`AxisManager` 负责单轴控制：

```cpp
enableServo(bool enable)
whenMove2AbsPosition(float vel, float pos)
moveForward(float vel, bool checked)
moveReverse(float vel, bool checked)
setHome()
stopSport(bool checked)
immediateStop(bool checked)
onRealTimeout()
```

`onRealTimeout()` 周期读取：

- 当前位置。
- 当前速度。
- 32 个状态位。

状态变化后通过信号更新 UI：

```cpp
emit sendPositionAndSpeed(m_currentPosition, m_currentSpeed);
emit sendTextState(messageAxis, messageMotion);
emit sendRailStatus(MY_COLOR::GREEN, m_axis);
```

绝对定位流程：

```text
检查 PLC 和使能状态
        ↓
写目标位置 RegR::AbsPosition
        ↓
写速度 RegR::AbsSpeed
        ↓
AbsPositionCommand 先 false 再 true
        ↓
轮询状态位 AbsolutePosHolding / AbsolutePosComplete
        ↓
发 absoluteMoveFinished
```

调三轴时最容易出错的是：

- 寄存器地址偏移。
- float 拆两个 `quint16` 的高低字顺序。
- 命令线圈是否需要脉冲式触发。
- 急停、停止、复位和使能之间的状态顺序。
- UI 线程直接操作设备对象导致线程混乱。

### 25.16 点云调试应该看哪些文件

点云相关工具：

```text
src/utils/pointCloud/PointCloudFunc.h
src/utils/pointCloud/PointCloudFunc.cpp
src/utils/pointCloud/SeamConcavityExtractor.*
```

常用能力：

```cpp
transformSinglePoint
transformPointCloud
transformLine
transformPlane
transformCylinder
passthroughFilter
statisticFilter
pointcloudVoxelDownsampling
projectCloudToPlane
projectCloudToCylinder
lineCloudEndPoints
getPoint2LineDis
projPoint2Line
```

点云排错时建议按这个顺序看：

1. 原始点云有没有点。
2. 点云单位是否是 mm。
3. 点云坐标系是相机还是机器人。
4. 点云是否经过了 `Trans_c`、手眼矩阵或末端到基座矩阵。
5. 滤波阈值是否把点全部过滤掉。
6. RANSAC 阈值是否过小。
7. 圆柱轴、平面法向、直线方向是否归一化。
8. 拟合结果是否保存到 `WeldSeamInfo` 对应字段。
...
项目里有很多保存调试数据的路径，例如：

```text
./data/common/pointCloud.pcd
./data/common/nonPlanePointCloud.pcd
./data/common/CommonPC.pcd
./data/seamDetWithSeg/YYYYMMDD/*.pcd
```

这些文件比单看日志更直观。点云问题优先打开 PCD 看，而不是只猜参数。

### 25.17 深度学习模块在项目中的边界

深度学习模块主要在：

```text
src/deepLearning/objectDetect
src/deepLearning/segment
src/workpieceCoarseLocalization/src/yoloInference
```

它们在系统中承担的是：

- 粗定位：识别工件区域和焊缝区域。
- 结构光精扫前：给出焊缝区域 mask 或框，减少重建范围。
- 分割修正：给出焊缝 mask，用几何约束反算三维点。

不要把深度学习模块理解成“直接给机器人轨迹”。本项目里真正能让机器人执行的仍然是几何结果：

```text
mask / box
        ↓
点云 / 平面 / 圆柱 / 直线
        ↓
焊缝端点或曲线点
        ↓
robotPose
```

### 25.18 新增一种焊缝检测算法的路线

如果要新增一种龙门支架焊缝类型，推荐按这个路线：

1. 在 `WeldSeamInfo.h` 增加 `WELD_AREA_TYPE` 和 `WELD_TYPE`。
2. 新建检测类，继承 `AbstractSeamDet`。
3. 实现 `solveSeamsEndPoints(...)`。
4. 在 `SeamDetWithPointCloud::initGantrayFrameSeamsDet()` 里注册工厂函数。
5. 在 `PointCloudReconstruction::makeMaskForSeamsDetToGF()` 里把模型类别映射到新的区域类型。
6. 在轨迹规划里处理新的 `WELD_TYPE`。
7. 在 `write2File()` 里定义焊接动作、速度、电流、电压和起收弧点。
8. 保存 PCD、图像、日志，先离线验证，再上设备。

最关键的是第 3 步和第 6 步：

```text
检测算法解决“焊缝在哪里”
轨迹规划解决“焊枪怎么过去、怎么焊”
```

这两个责任不要混在一个类里。

### 25.19 新人最推荐的源码阅读顺序

如果目标是能改这个项目，而不是只看懂语法，建议按这个顺序：

1. `main.cpp`  
   看程序如何启动，哪些元类型需要跨线程传递。

2. `src/ui/WeldingMainWindow.*`  
   看 UI 按钮如何发信号，不要深究界面布局。

3. `src/weldingSystem/RailWeldingSystem.*`  
   画出模块初始化和信号连接图。

4. `src/utils/common/WeldSeamInfo.h`  
   把每个字段对应到处理阶段。

5. `src/structLightCamera/StructLightCamera.*`  
   看设备采集和重建触发。

6. `src/structLightCamera/reconstruction/PointCloudReconstruction.*`  
   看结构光点云怎么生成。

7. `src/seamDetWithPointCloud/SeamDetWithPointCloud.*`  
   看焊缝算法如何按类型分发。

8. `src/seamDetWithPointCloud/gantrayFrameDet/...`  
   选一个具体焊缝类型深入读，不要同时读所有算法。

9. `src/seamDetWithSeg/SeamDetWithSeg.*`  
   看分割如何反投影到三维并融合点云结果。

10. `src/robotTrajectoryPlanning/GantrayFrameTrajectoryPlanning.*`  
    看点云结果如何变成机器人姿态和路点文件。

11. `src/rail/concrete_axis/AxisManager.*`  
    看三轴状态和寄存器读写。

12. `src/robotFactory/...`  
    看机器人品牌差异如何被抽象接口屏蔽。

### 25.20 一条焊缝从采集到焊接的完整生命周期

用一句长流程把项目串起来：

```text
粗定位相机获取工件图像
        ↓
YOLO 找工件和焊缝区域
        ↓
FittingWorkpieceCoordinate 转世界/基座坐标并规划拍照位
        ↓
RailWeldingSystem 控制机器人和三轴移动到拍照位置
        ↓
结构光相机和投影仪采集相移/格雷码图
        ↓
PointCloudReconstruction 解相位并重建点云
        ↓
目标检测/分割生成焊缝区域 mask
        ↓
重建每个焊缝区域的局部点云
        ↓
SeamDetWithPointCloud 按焊缝类型拟合几何特征
        ↓
/./SeamDetWithSeg 用 mask 反算三维点并修正端点
        ↓
GantrayFrameTrajectoryPlanning 转机器人基坐标、补偿、算姿态
        ↓
碰撞检测和摆焊参考点计算
        ↓
write2File 生成 SeamCoordinate.txt
        ↓
机器人执行焊接，三轴配合移动
        ↓
UI、日志、错误数据持续反馈
```

真正掌握这个项目的标志，不是能背出某个函数，而是看到一条异常日志时，能判断它属于这条链路的哪一段。
