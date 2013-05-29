//#define WINDOWS

#ifdef WINDOWS
#define _CRT_SECURE_NO_DEPRECATE 1
#define _USE_MATH_DEFINES
#include <winsock2.h>
#else
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#endif

#include <string>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include "math.h"
#include <sstream>
//#define NDEBUG
#include <assert.h>
#include <stack>
#include <queue>

#define MY_DEBUG true

using namespace std;

#define MAX_OBSTACLE_CORNERS 10
const int kBufferSize = 1024;
const bool DEBUG_S = false;

typedef struct team_t {
	string color;
	int count;
	double base_corner[4][2];
} team_t;

typedef struct obstacle_t {
	double o_corner[MAX_OBSTACLE_CORNERS][2];
} obstacle_t;

typedef struct flag_t {
	string color;
	string poss_color;
	double pos[2];
} flag_t;

typedef struct shot_t {
	double pos[2];
	double velocity[2];
} shot_t;

typedef struct tank_t {
	int index;
	string callsign;
	string status;
	int shots_avail;
	double time_to_reload;
	string flag;
	double pos[2];
	double angle;
	double velocity[2];
	double angvel;
} tank_t;

typedef struct otank_t {
	string callsign;
	string color;
	string status;
	string flag;
	double pos[2];
	double angle;
} otank_t;

typedef struct constant_t {
	string name;
	string value;
} constant_t;

typedef struct direction_t {
	double x;
	double y;
} direction_t;

typedef struct coordinate_t {
	double x;
	double y;
} coordinate_t;

typedef struct grid_t {
	int left;
	int top;
	int width;
	int height;
	vector<vector<int> > obstacles;
} grid_t;

typedef struct direction_grid_t {
	int left;
	int top;
	int width;
	int height;
	vector<vector<direction_t> > contents;
} direction_grid_t;

typedef struct weight_grid_t {
	int left;
	int top;
	int width;
	int height;
	vector<vector<double> > weights;
} weight_grid_t;

typedef struct prioritizable_node_t {
	double x;
	double y;
	double prev_x;
	double prev_y;
	double cost;
	double heuristic;
	bool operator< (const prioritizable_node_t& other) const
	{
		double difference = (cost + heuristic) - (other.cost + other.heuristic);
		return difference > 0;
	};
} prioritizable_node_t;

typedef struct tank_brain_t {
	long last_updated_s; // _s is for seconds 
	coordinate_t current_goal;
	direction_t heading;
} tank_brain_t;

class SplitString {
	vector <string> MyVector;
	string MyString;
public:
	SplitString(string str) {
		MyString=str;
		MyVector.begin();
	}

	vector <string> Split() {
		MyVector.clear();
		size_t LastLoc = -1;
		size_t CurLoc = MyString.find(" ", 0);
		while (CurLoc != string::npos) {
			string currentString = MyString.substr(LastLoc+1, CurLoc-LastLoc-1);
			if (!currentString.empty())
			{
				MyVector.push_back(MyString.substr(LastLoc+1, CurLoc-LastLoc-1));
			}
			LastLoc=CurLoc;
			CurLoc = MyString.find(" ", LastLoc+1);
		}

		MyVector.push_back(MyString.substr(LastLoc+1, MyString.size()-LastLoc));
		return MyVector;
	}
};

class BZRC {
	const char *pcHost;
	int nPort;
	bool debug;
	bool InitStatus;
	char ReplyBuffer[kBufferSize];
	int LineFeedPos;
	int Start;

#ifdef WINDOWS
	SOCKET sd;
#else
	int sd;
#endif

	// Initializing connection.
	int Init() {
		ResetReplyBuffer();
		Start=0;

#ifdef WINDOWS
		// Start Winsock up
		if(int nCode = WinSockInit() != 0) {
			cerr << "WSAStartup() returned error code " << nCode << "." << endl;
			return 1;
		}

		// Find the server's address
		u_long nRemoteAddress = LookupAddress(pcHost);
		if(nRemoteAddress == INADDR_NONE) {
			cerr << "Host cannot be found." << endl;
			return 1;
		}

		// Connect to the server
		sd = EstablishConnection(nRemoteAddress, htons(nPort));
		if(sd == INVALID_SOCKET) {
			cerr << WSAGetLastError() << endl;
			return 1;
		}
#else
		struct addrinfo *infop = NULL;
		struct addrinfo hint;

		memset(&hint, 0, sizeof(hint));
		hint.ai_family = AF_INET;
		hint.ai_socktype = SOCK_STREAM;

		char port[10];
		snprintf(port, 10, "%d", nPort); // originally snprintf(....)

		if (getaddrinfo(pcHost, port, &hint, &infop) != 0) {
			perror("Couldn't lookup host.");
			return 1;
		}

		if ((sd = socket(infop->ai_family, infop->ai_socktype,
						infop->ai_protocol)) < 0) {
			perror("Couldn't create socket.");
			return 1;
		}

		if (connect(sd, infop->ai_addr, infop->ai_addrlen) < 0) {
			perror("Couldn't connect.");
			//close(sd);
		}

		freeaddrinfo(infop);
#endif


		if(HandShake()==1) {
			cerr << "Handshake failed!" << endl;
			return 1;
		}

		return 0;		
	}

#ifdef WINDOWS
	// Initialize WinSock
	int WinSockInit() {
		LPWSADATA lpmyWSAData;
		WORD VersionReqd;
		VersionReqd = MAKEWORD(2,0);
		lpmyWSAData = (LPWSADATA)malloc(sizeof(WSADATA));
		int Status = WSAStartup(VersionReqd, lpmyWSAData);
		if(Status != 0) return 0;
		if(LOBYTE(lpmyWSAData->wVersion) != 2 || HIBYTE(lpmyWSAData->wVersion) != 0) {
			WSACleanup();
			free(lpmyWSAData);
			return 0;
		}
		free(lpmyWSAData);
		return Status;
	}

	// Lookup IP address
	u_long LookupAddress(const char *pcHost) {
			u_long nRemoteAddr = inet_addr(pcHost);
			if (nRemoteAddr == INADDR_NONE) {
				// pcHost isn't a dotted IP, so resolve it through DNS
				hostent* pHE = gethostbyname(pcHost);
				if (pHE == 0) {
					return INADDR_NONE;
				}
				nRemoteAddr = *((u_long*)pHE->h_addr_list[0]);
			}
			return nRemoteAddr;
		}

	// Establish connection
	SOCKET EstablishConnection(u_long nRemoteAddr, u_short nPort) {
		// Create a stream socket
		SOCKET MySocket = socket(AF_INET, SOCK_STREAM, 0);
		if (MySocket != INVALID_SOCKET) {
			sockaddr_in sinRemote;
			sinRemote.sin_family = AF_INET;
			sinRemote.sin_addr.s_addr = nRemoteAddr;
			sinRemote.sin_port = nPort;
			if (connect(MySocket, (sockaddr*)&sinRemote, sizeof(sockaddr_in)) == SOCKET_ERROR) {
				MySocket = INVALID_SOCKET;
			}
		}
		return MySocket;
	}
#endif

	// Send line to server
	int SendLine(const char *LineText) {
		int Length=(int)strlen(LineText);
		char Command[kBufferSize];
		strcpy(Command, LineText);
		Command[Length]='\n';
		Command[Length+1]='\0';
		if(debug) cout << Command;
		if (send(sd, Command, Length+1, 0) >= 0) {
			return 0;
		}
		else {
			return 1;
		}
	}
	// Read line back from server
	int ReadReply(char *Reply)
	{
		char acReadBuffer[kBufferSize];

		int nNewBytes = recv(sd, acReadBuffer, kBufferSize, 0);
		if (nNewBytes < 0) {
			return -1;
		}
		else if (nNewBytes == 0) {
			cerr << "Connection closed by peer." << endl;
			cerr << "Closing..." << endl;
			exit(0);
			return 0;
		}
		
		memcpy(Reply, &acReadBuffer, nNewBytes);
		if(nNewBytes!=kBufferSize) {
			Reply[nNewBytes]='\0';
		}

		return nNewBytes;
	}

	// Only read one line of text from ReplyBuffer
	void ReadLine(char *LineText) {
		memset(LineText, '\0', kBufferSize);
		// Only read more from server when done wiht current ReplyBuffer
		if(strlen(ReplyBuffer)==0) {
			char *Reply;
			Reply = ReplyBuffer;
			ReadReply(Reply);
		}
		int i=0;
		bool done=false;
		while(!done) {
			for(i=LineFeedPos+1; (i<kBufferSize && ReplyBuffer[i]); i++) {
				if(ReplyBuffer[i]=='\n') {
					LineText[i-LineFeedPos-1+Start]='\0';
					LineFeedPos=i;
					Start=0;
					done=true;
					break;
				}
				LineText[i-LineFeedPos-1+Start]=ReplyBuffer[i];
			}
			if(!done) {
					Start = (int)strlen(LineText);
					ResetReplyBuffer();	
					char *Reply;
					Reply = ReplyBuffer;
					ReadReply(Reply);
			}
			else {
				if(ReplyBuffer[i]=='\0') {
					done=true;
					Start=0;
					ResetReplyBuffer();
				}
			}
		}
	}

	// Reset the ReplyBuffer
	void ResetReplyBuffer() {
		memset(ReplyBuffer, '\0', kBufferSize);
		LineFeedPos=-1;
	}

	// Perform HandShake with the server
	int HandShake() {
		char str[kBufferSize];
		char *LineText;
		LineText=str;
		ReadLine(LineText);
		if(debug) cout << LineText << endl;
		if (!strcmp(LineText, "bzrobots 1")) {
			const char * Command="agent 1";
			int temp=SendLine(Command);
			if(temp==1) 
				return 1;
			else
				ResetReplyBuffer();
				return 0;
		}
		else
			return 1;
	}

	// Read line into vector
	vector <string> ReadArr() {
		char str[kBufferSize];
		char *LineText=str;
		ReadLine(LineText);
		if(strlen(LineText)!=0) {
			if(debug) cout << LineText << endl;
		}
		while(strlen(LineText)==0) {
			ReadLine(LineText);
			if(debug) cout << LineText << endl;
		}
		SplitString ss=SplitString(LineText);
		return ss.Split();
	}
	// Read Acknowledgement
	void ReadAck() {
		vector <string> v=ReadArr();
		if(v.at(0)!="ack") {
			cout << "Didn't receive ack! Exit!" << endl;
			exit(1);
		}
	}
	// Read "ok"
	bool ReadBool() {
		vector <string> v=ReadArr();
		if(v.at(0)=="ok") {
			return true;
		}
		else if(v.at(0)=="fail"){
			if(debug) cout << "Received fail. Exiting!" << endl;
			return false;
		}
		else {
			if(debug) cout << "Something went wrong. Exiting!" << endl;
			return false;
		}
	}
	// Receive and print another line
	void PrintLine() {
		char str[kBufferSize];
		char *LineText=str;
		ReadLine(LineText);
		if(debug) cout << LineText << endl;
	}
public:
	BZRC(const char *host, int port, bool debug_mode) {
		pcHost = host;
		nPort = port;
		debug = debug_mode;
		if(Init()) {
			cout << "BZRC initialization failed." << endl;
			InitStatus=false;
			Close();
		}
		else {
			InitStatus=true;
		}
	}

	// Self check
	int GetPort(){return nPort;}
	const char *GetHost() {return pcHost;}
	bool GetStatus() {return InitStatus;}
	// Commands:
	bool shoot(int index) {
		// Perform a shoot request.
		char char_buff[20];
		sprintf(char_buff, " %d", index);	
		string str_buff="shoot";
		str_buff.append(char_buff);
		const char *Command = str_buff.c_str();
		SendLine(Command);
		ReadAck();
		if(ReadBool()) {
			return true;
		}
		else {
			return false;
		}
	}

	bool speed(int index, double value) {
		// Set the desired speed to the specified value.
		char char_buff[20];
		sprintf(char_buff, " %d", index);	
		string str_buff="speed";
		str_buff.append(char_buff);
		sprintf(char_buff, " %f", value);
		str_buff.append(char_buff);
		const char *Command = str_buff.c_str();
		SendLine(Command);
		ReadAck();
		if(ReadBool()) {
			return true;
		}
		else {
			return false;
		}
	}

	bool angvel(int index, double value) {
		// Set the desired angular velocity to the specified value.
		char char_buff[20];
		sprintf(char_buff, " %d", index);	
		string str_buff="angvel";
		str_buff.append(char_buff);
		sprintf(char_buff, " %f", value);
		str_buff.append(char_buff);
		const char *Command = str_buff.c_str();
		SendLine(Command);
		ReadAck();
		if(ReadBool()) {
			return true;
		}
		else {
			return false;
		}
	}

	bool accelx(int index, double value) {
		// Set the desired accelaration in x axis to the specified value in hovertank mode.
		char char_buff[20];
		sprintf(char_buff, " %d", index);	
		string str_buff="accelx";
		str_buff.append(char_buff);
		sprintf(char_buff, " %f", value);
		str_buff.append(char_buff);
		const char *Command = str_buff.c_str();
		SendLine(Command);
		ReadAck();
		if(ReadBool()) {
			return true;
		}
		else {
			return false;
		}
	}	
	bool accely(int index, double value) {
		// Set the desired accelaration in x axis to the specified value in hovertank mode.
		char char_buff[20];
		sprintf(char_buff, " %d", index);	
		string str_buff="accely";
		str_buff.append(char_buff);
		sprintf(char_buff, " %f", value);
		str_buff.append(char_buff);
		const char *Command = str_buff.c_str();
		SendLine(Command);
		ReadAck();
		if(ReadBool()) {
			return true;
		}
		else {
			return false;
		}
	}

	// Information Request:
	bool get_teams(vector <team_t> *AllTeams) {
		//Request a list of teams.
		SendLine("teams");
		ReadAck();
		vector <string> v=ReadArr();
		if(v.at(0)!="begin") {
			return false;
		}
		v.clear();
		v=ReadArr();
		int i=0;
		while(v.at(0)=="team") {
			team_t MyTeam;
			MyTeam.color=v.at(1);
			MyTeam.count=atoi(v.at(2).c_str());
			MyTeam.base_corner[0][0]=atof(v.at(3).c_str());
			MyTeam.base_corner[0][1]=atof(v.at(4).c_str());
			MyTeam.base_corner[1][0]=atof(v.at(5).c_str());
			MyTeam.base_corner[1][1]=atof(v.at(6).c_str());
			MyTeam.base_corner[2][0]=atof(v.at(7).c_str());
			MyTeam.base_corner[2][1]=atof(v.at(8).c_str());
			MyTeam.base_corner[3][0]=atof(v.at(9).c_str());
			MyTeam.base_corner[3][1]=atof(v.at(10).c_str());
			AllTeams->push_back(MyTeam);
			v.clear();
			v=ReadArr();
			i++;
		}
		if(v.at(0)!="end") {
			return false;
		}
		return true;
	}

	/**
	 * We added this
	 */
	bool get_bases(vector <team_t> *AllTeams) {
		//Request a list of teams.
		SendLine("bases");
		ReadAck();
		vector <string> v=ReadArr();
		if(v.at(0)!="begin") {
			return false;
		}
		v.clear();
		v=ReadArr();
		int i=0;
		while(v.at(0)=="base") {
			team_t MyTeam;
			MyTeam.color=v.at(1);
			MyTeam.base_corner[0][0]=atof(v.at(2).c_str());
			MyTeam.base_corner[0][1]=atof(v.at(3).c_str());
			MyTeam.base_corner[1][0]=atof(v.at(4).c_str());
			MyTeam.base_corner[1][1]=atof(v.at(5).c_str());
			MyTeam.base_corner[2][0]=atof(v.at(6).c_str());
			MyTeam.base_corner[2][1]=atof(v.at(7).c_str());
			MyTeam.base_corner[3][0]=atof(v.at(8).c_str());
			MyTeam.base_corner[3][1]=atof(v.at(9).c_str());
			AllTeams->push_back(MyTeam);
			v.clear();
			v=ReadArr();
			i++;
		}
		if(v.at(0)!="end") {
			return false;
		}
		return true;
	}

	bool get_obstacles(vector <obstacle_t> *AllObstacles) {
		// Request a list of obstacles.
		SendLine("obstacles");
		ReadAck();
		vector <string> v=ReadArr();
		if(v.at(0)!="begin") {
			return false;
		}
		v.clear();
		v=ReadArr();
		int i=0;
		while(v.at(0)=="obstacle") {
			obstacle_t MyObstacle;
			int j=0;
			while(j+2<(int)v.size()) {
				MyObstacle.o_corner[j/2][0]=atof(v.at(j+1).c_str());
				MyObstacle.o_corner[j/2][1]=atof(v.at(j+2).c_str());
				j=j+2;
			}
			AllObstacles->push_back(MyObstacle);
			v.clear();
			v=ReadArr();
			i++;
		}
		if(v.at(0)!="end") {
			return false;
		}
		return true;
	}

	bool get_flags(vector <flag_t> *AllFlags) {
		// Request a list of flags.
		SendLine("flags");
		ReadAck();
		vector <string> v=ReadArr();
		if(v.at(0)!="begin") {
			return false;
		}
		v.clear();
		v=ReadArr();
		int i=0;
		while(v.at(0)=="flag") {
			flag_t MyFlag;
			MyFlag.color=v.at(1);
			MyFlag.poss_color=v.at(2);
			MyFlag.pos[0]=atof(v.at(3).c_str());
			MyFlag.pos[1]=atof(v.at(4).c_str());
			AllFlags->push_back(MyFlag);
			v.clear();
			v=ReadArr();
			i++;
		}
		if(v.at(0)!="end") {
			return false;
		}
		return true;
	}

	bool get_shots(vector <shot_t> *AllShots) {
		// Request a list of shots.
		SendLine("shots");
		ReadAck();
		vector <string> v=ReadArr();
		if(v.at(0)!="begin") {
			return false;
		}
		v.clear();
		v=ReadArr();
		int i=0;
		while(v.at(0)=="shot") {
			shot_t MyShot;
			MyShot.pos[0]=atof(v.at(1).c_str());
			MyShot.pos[1]=atof(v.at(2).c_str());
			MyShot.velocity[0]=atof(v.at(3).c_str());
			MyShot.velocity[1]=atof(v.at(4).c_str());
			AllShots->push_back(MyShot);
			v.clear();
			v=ReadArr();
			i++;
		}
		if(v.at(0)!="end") {
			return false;
		}
		return true;
	}

	bool get_mytanks(vector <tank_t> *AllMyTanks) {
		// Request a list of our robots.
		SendLine("mytanks");
		ReadAck();
		vector <string> v=ReadArr();
		if(v.at(0)!="begin") {
			return false;
		}
		v.clear();
		v=ReadArr();
		int i=0;
		while(v.at(0)=="mytank") {
			tank_t MyTank;
			MyTank.index=atoi(v.at(1).c_str());
			MyTank.callsign=v.at(2);
			MyTank.status=v.at(3);
			MyTank.shots_avail=atoi(v.at(4).c_str());
			MyTank.time_to_reload=atof(v.at(5).c_str());
			MyTank.flag=v.at(6);
			MyTank.pos[0]=atof(v.at(7).c_str());
			MyTank.pos[1]=atof(v.at(8).c_str());

			//cout << MyTank.pos[0];

			//printf("Received tank at position: %f, %f.\n", MyTank.pos[0], MyTank.pos[1]);

			MyTank.angle=atof(v.at(9).c_str());
			MyTank.velocity[0]=atof(v.at(10).c_str());
			MyTank.velocity[1]=atof(v.at(11).c_str());
			MyTank.angvel=atof(v.at(12).c_str());
			AllMyTanks->push_back(MyTank);
			v.clear();
			v=ReadArr();
			i++;
		}
		if(v.at(0)!="end") {
			if(debug) cout << v.at(0) << endl;
			return false;
		}
		return true;
	}

	bool get_othertanks(vector <otank_t> *AllOtherTanks) {
		// Request a list of tanks that aren't our robots.
		SendLine("othertanks");
		ReadAck();
		vector <string> v=ReadArr();
		if(v.at(0)!="begin") {
			return false;
		}
		v.clear();
		v=ReadArr();
		int i=0;
		while(v.at(0)=="othertank") {
			otank_t OtherTank;
			OtherTank.callsign=v.at(1);
			OtherTank.color=v.at(2);
			OtherTank.status=v.at(3);
			OtherTank.flag=v.at(4);
			OtherTank.pos[0]=atof(v.at(5).c_str());
			OtherTank.pos[1]=atof(v.at(6).c_str());
			OtherTank.angle=atof(v.at(7).c_str());
			AllOtherTanks->push_back(OtherTank);
			v.clear();
			v=ReadArr();
			i++;
		}
		if(v.at(0)!="end") {
			return false;
		}
		return true;
	}

	bool get_occgrid(grid_t& world_grid) {
		SendLine("occgrid 0");
		ReadAck();
		vector <string> v=ReadArr();
		if(v.at(0)!="begin") {
			if (DEBUG_S) {
				printf("First word is occgrid response is not 'begin'. It was %s. Returning...\n", v.at(0).c_str());
			}
			return false;
		}
		v.clear();

		// Get location of top-left corner of grid
		v=ReadArr();
		int location_of_comma = v.at(1).find(",");
		world_grid.left = atoi(v.at(1).substr(0, location_of_comma).c_str());
		world_grid.top = atoi(v.at(1).substr(location_of_comma + 1, (v.at(1).length() - 1 - location_of_comma)).c_str()); // TODO
		if (DEBUG_S) {
			printf("Occ grid string: %s\n", v.at(1).c_str());
		}
		v.clear();
		
		if (DEBUG_S) {
			printf("World grid left = %d\n", world_grid.left);
			printf("World grid top = %d\n", world_grid.top);
		}
		
		// Get dimensions of grid
		v=ReadArr();
		int location_of_x = v.at(1).find("x");
		world_grid.width = atoi(v.at(1).substr(0, location_of_x).c_str());
		world_grid.height = atoi(v.at(1).substr(location_of_x + 1, (v.at(1).length() - 1 - location_of_x)).c_str());
		v.clear();

		if (DEBUG_S) {
			printf("World grid width = %d\n", world_grid.width);
			printf("World grid height = %d\n", world_grid.height);
		}
		
		// Top-left corner of grid corresponds to bottom-left corner of world
		// Grid comes in as:
		// ----Height----
		// | ++++++++++++
		// | + 			+
		// W +			+
		// i +			+
		// d +			+
		// t +			+
		// h +			+
		// | +			+
		// | ++++++++++++
		// 
		// And I store it in grid_t as:
		// --------Width--------
		// H +++++++++++++++++++
		// e +				   +
		// i +				   +
		// g +				   +
		// h +				   +
		// t +++++++++++++++++++

		vector<string> old_world;
		old_world.resize(world_grid.width);

		if (DEBUG_S) {
			printf("Importing grid...\n");
		}
		// Get grid lines (each line corresponds to a different x-coordinate)
		v=ReadArr();
		for (int width_n = 0; width_n < world_grid.width; width_n++) {
			old_world.at(width_n) = v.at(0);
			v.clear();
			v=ReadArr();
			if (DEBUG_S) {
				cout << v.at(0).c_str() << endl;
			}
		}

		world_grid.obstacles.resize(world_grid.height);
		for (int height_n = 0; height_n < world_grid.height; height_n++) {
			for (int width_n = world_grid.width - 1; width_n >= 0; width_n--) {
				//int old_world_digit = atoi(&old_world.at(width_n).at(height_n));
				char old_world_char = old_world.at(height_n).at(width_n);
				assert(old_world_char == '1' || old_world_char == '0');
				int old_world_digit = old_world_char - 48; //48 is ascii value of 0
				world_grid.obstacles.at(height_n).push_back(old_world_digit);
			}
		}

		// We need to rotate the grid counter-clockwise by 90 degrees because that's how it gets sent to us for some reason.'
		//vector<vector<int> > rotated_grid;
		//for ()
		//)
		
				
		return true;
	}

	bool get_tank_vision_grid(grid_t& vision_grid, int tank_n) {
		volatile int i = 5 / 0;
		string query_message = "occgrid ";
		char buffer[4];
		sprintf(buffer, "%d", tank_n);
		query_message.append(buffer);
		SendLine(query_message.c_str());
		ReadAck();
		vector <string> v=ReadArr();
		if(v.at(0)!="begin") {
			if (DEBUG_S) {
				printf("First word is occgrid response is not 'begin'. It was %s. Returning...\n", v.at(0).c_str());
			}
			return false;
		}
		v.clear();

		// Get location of top-left corner of grid
		v=ReadArr();
		int location_of_comma = v.at(1).find(",");
		vision_grid.left = atoi(v.at(1).substr(0, location_of_comma).c_str());
		vision_grid.top = atoi(v.at(1).substr(location_of_comma + 1, (v.at(1).length() - 1 - location_of_comma)).c_str()); // TODO
		if (DEBUG_S) {
			printf("Occ grid string: %s\n", v.at(1).c_str());
		}
		v.clear();
		
		if (DEBUG_S) {
			printf("World grid left = %d\n", vision_grid.left);
			printf("World grid top = %d\n", vision_grid.top);
		}
		
		// Get dimensions of grid
		v=ReadArr();
		int location_of_x = v.at(1).find("x");
		vision_grid.width = atoi(v.at(1).substr(0, location_of_x).c_str());
		vision_grid.height = atoi(v.at(1).substr(location_of_x + 1, (v.at(1).length() - 1 - location_of_x)).c_str());
		v.clear();

		if (DEBUG_S) {
			printf("World grid width = %d\n", vision_grid.width);
			printf("World grid height = %d\n", vision_grid.height);
		}
		
		// Top-left corner of grid corresponds to bottom-left corner of world
		// Grid comes in as:
		// ----Height----
		// | ++++++++++++
		// | + 			+
		// W +			+
		// i +			+
		// d +			+
		// t +			+
		// h +			+
		// | +			+
		// | ++++++++++++
		// 
		// And I store it in grid_t as:
		// --------Width--------
		// H +++++++++++++++++++
		// e +				   +
		// i +				   +
		// g +				   +
		// h +				   +
		// t +++++++++++++++++++

		vector<string> old_world;
		old_world.resize(vision_grid.width);

		if (DEBUG_S) {
			printf("Importing grid...\n");
		}
		// Get grid lines (each line corresponds to a different x-coordinate)
		v=ReadArr();
		for (int width_n = 0; width_n < vision_grid.width; width_n++) {
			old_world.at(width_n) = v.at(0);
			v.clear();
			v=ReadArr();
			if (DEBUG_S) {
				cout << v.at(0).c_str() << endl;
			}
		}

		vision_grid.obstacles.resize(vision_grid.height);
		for (int height_n = 0; height_n < vision_grid.height; height_n++) {
			for (int width_n = vision_grid.width - 1; width_n >= 0; width_n--) {
				//int old_world_digit = atoi(&old_world.at(width_n).at(height_n));
				char old_world_char = old_world.at(height_n).at(width_n);
				assert(old_world_char == '1' || old_world_char == '0');
				int old_world_digit = old_world_char - 48; //48 is ascii value of 0
				vision_grid.obstacles.at(height_n).push_back(old_world_digit);
			}
		}

		// We need to rotate the grid counter-clockwise by 90 degrees because that's how it gets sent to us for some reason.'
		//vector<vector<int> > rotated_grid;
		//for ()
		//)
		
				
		return true;
	}

	void print_grid(grid_t grid) {
		assert(grid.height != 0 && grid.width != 0);
		for (int row_n = 0; row_n < grid.height; row_n++) {
			for (int col_n = 0; col_n < grid.width; col_n++) {
				printf("%d", grid.obstacles[row_n][col_n]);
			}
			printf("\n");
		}
		if (DEBUG_S) {
			printf("Exiting print_grid() function...\n");
		}
	}

	bool get_constants(vector <constant_t> *AllConstants) {
		// Request a dictionary of game constants.
		SendLine("constants");
		ReadAck();
		vector <string> v=ReadArr();
		if(v.at(0)!="begin") {
			return false;
		}
		v.clear();
		v=ReadArr();
		int i=0;
		while(v.at(0)=="constant") {
			constant_t MyConstant;
			MyConstant.name=v.at(1);
			MyConstant.value=v.at(2);
			AllConstants->push_back(MyConstant);
			v.clear();
			v=ReadArr();
			i++;
		}
		if(v.at(0)!="end") {
			return false;
		}
		return true;
	}

	int Close() {
#ifdef WINDOWS
		closesocket(sd);
		WSACleanup();
#else
		//close(sd);
#endif
		return 0;
	}

};

// Prototypes
void world_init(BZRC *my_team);
void robot_pre_update(BZRC *my_team);
void robot_update(BZRC *my_team);
void robot_post_update(BZRC *my_team);
void print_potentials(double goal_x, double goal_y, const char* file_name);
direction_t get_potential_field(double tank_x, double tank_y, double goal_x, double goal_y);
flag_t get_opposing_flag(tank_t tank, BZRC* my_team);
direction_t calculate_attractor(double tank_x, double tank_y, double goal_x, double goal_y);
direction_t calculate_obstacle_repulsion(double tank_x, double tank_y, obstacle_t obstacle);
void print_grid(const char*);
void define_constants();
void store_enemy_tanks_coors(BZRC* my_team);
void store_green_flag(BZRC* my_team);
void store_red_tank(BZRC* my_team);
vector<vector<bool> * > * get_empty_bool_grid(int width, int height);
stack<coordinate_t> * recursive_depth_first_search(int target_x, int target_y, int current_x, int current_y, vector<vector<bool> * > * visited_locations, stack<coordinate_t> * path_so_far, int max_depth);

void fill_visited_grid(int width, int height);
void fill_directional_grid(int width, int height);

void display_path(const char* filename, stack<coordinate_t> * path);

stack<coordinate_t> * breadth_first_search(int target_x, int target_y, int start_x, int start_y);

void display_wavefront(const char* filename, queue<coordinate_t> locations);

stack<coordinate_t> * iterative_deepening_depth_first_search(int target_x, int target_y, int current_x, int current_y);
stack<coordinate_t> * iddfs_recursor(int target_x, int target_y, int current_x, int current_y, stack<coordinate_t> * path_so_far, int max_depth);
void print_visited_grid(const char* filename);

void populate_tank_grid();
void print_tank_weights(const char* filename);

stack<coordinate_t> * best_first_search(int target_x, int target_y, int start_x, int start_y, bool use_heuristic);
double generate_heuristic(int target_x, int target_y, int start_x, int start_y);
void display_pqueue_wavefront(const char* filename, priority_queue<prioritizable_node_t> locations);

bool has_adjacent_occupied(int x, int y);

void set_tank_heading(int tank_n, stack<coordinate_t> * path, BZRC* my_team);
void keep_tank_on_course(int tank_n, BZRC* my_team);

void populate_world_grid(int size);

void update_tank_vision(BZRC* my_team);

