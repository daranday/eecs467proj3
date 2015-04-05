#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <fstream>
#include <iostream>
#include <vector>

// drawables
#include "vx/vxo_drawables.h"

// common
#include "common/getopt.h"
#include "common/pg.h"
#include "common/zarray.h"


#include "apps/eecs467_util.h"    // This is where a lot of the internals live


//lcm
#include "lcm/lcm-cpp.hpp"
#include "lcmtypes/ttt_turn_t.hpp"

using namespace std;

//LCM setup
pthread_mutex_t md;
lcm::LCM lcm_inst;

void *lcm_comm(void *input){
  while(true){
    lcm_inst.handle();
  }
  return NULL;
}

struct move_t{
  int64_t utime;
  int32_t turn;

  move_t(){
    turn = 0;
    utime = 0;
  }
  
};

struct comms{
  int our_turn;
  bool is_red;
  bool c_move;
  move_t current;

  comms(){
    c_move = false;
    our_turn = 0;
  }

  void ttt_turn_handler(const lcm::ReceiveBuffer* rbuf, const string &channel, const ttt_turn_t* msg){
    pthread_mutex_lock(&md);
    current.utime = msg->utime;
    current.turn = msg->turn;
    pthread_mutex_unlock(&md);
    can_move();
  }

  void can_move(){
    pthread_mutex_lock(&md);
    if(is_red){
        if(our_turn == current.turn){
            c_move = true;
        }
        else if(our_turn > current.turn)
            c_move= false;
    }
    else{
        if(our_turn == current.turn)
            c_move = false;
        else if(our_turn < current.turn){
            c_move = true;
        }
    }
    pthread_mutex_unlock(&md);
  }

  bool wait_turn(){
    bool ret;
    pthread_mutex_lock(&md);
    ret = c_move;
    c_move = false;
    pthread_mutex_unlock(&md);
    return ret;

  }

};

int main(){
	string junk;
	string player;
	string channel;
	string our_chan;
	cout << "what player are we (lower case)? \nenter: ";
	cin >> player;
	comms C;

	if(player == "green"){
		channel = "RED_TURN";
		our_chan = "GREEN_TURN";
		C.is_red = false;
	}	
	else {
		C.is_red = true;
		channel = "GREEN_TURN";
		our_chan = "RED_TURN";
		C.is_red = true;
	}

  cout << "subscribing to " << channel << endl;

	//LCM initialization
    lcm_inst.subscribe(channel, &comms::ttt_turn_handler, &C);

    pthread_t ttt_comm;
    pthread_create(&ttt_comm, NULL, lcm_comm, NULL);
    ttt_turn_t msg;
    ttt_turn_t *msg_ptr = &msg;
    //done with lcm initialization
    cout << " here " << endl;
    while(true){
      if(C.wait_turn()){
      	cout << "enter anything" << endl;
      	cin >> junk;

      	++C.our_turn;

      	
      }
      
      pthread_mutex_lock(&md);
      cout << "our turn: " << C.our_turn << " opponent turn: " << C.current.turn << endl;
      pthread_mutex_unlock(&md);

      msg_ptr->turn =  C.our_turn;
      usleep(50000);
      lcm_inst.publish(our_chan, msg_ptr);
    }



}