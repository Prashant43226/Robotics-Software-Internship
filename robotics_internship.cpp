#include <bits/stdc++.h>
using namespace std;

const signed inf = 0x3f3f3f3f;
int tot_swaps=0;
int tot_r=0;
////////////////////////////////////////////////
vector<vector<int>> travel_time;
mt19937 rng(chrono::steady_clock::now().time_since_epoch().count());
///////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
struct ROBOT{
	int curr_charge;
	int curr_cost;
	int mask_drops;
	int mask_pickups;
	int bot_key;
	int max_load;
	vector<int> order_list;
	vector<int>order_list_with_charging;
    vector<int>order_list_with_constraints;
    vector<int>order_list_with_constraint_charging;
    
	ROBOT(int bot_k){
		curr_charge = 100;
		curr_cost=0;
		bot_key= bot_k;
		mask_drops=(1<<bot_k);
		mask_pickups=(1<<bot_k);
		order_list = {2*bot_k, 2*bot_k+1};
		max_load=2;
		order_list_with_charging.clear();
	}
	
	// insert a given order optimally in the order_list without changing order of previous nodes.
	// return boolean value: True -> insertion successfull 
	//					 	 False -> NOT successfull
	// Note : tardiness cost addition remains 
	int pseudo_insert(int order, bool insert_in_list){
		int pickup_node = 2*order;
		int drop_node = 2*order+1;
		int insertion_cost = inf;             // pseudo_insertion_cost variable to be minimized.
		int n = order_list.size();  // size of the BOT order list currently.
		int load = 0; // load on the BOT (currently taken to be 0).
		
		int best_pickup_cost=inf, best_pickup_loc , best_drop_loc, best_pickup_loc_so_far;

		for(int i=1;i<n;i++){
			if(order_list[i]%2==0){
				load++;
			}else{
				load--;
			}
			if(load>=max_load){
				best_pickup_cost = inf;
				continue;
			} 
			// Initializing cost if the Pickup and Drop both are adjacent to each other ahead of the i-th node.
			int adj_insertion_cost = travel_time[order_list[i]][pickup_node] + travel_time[pickup_node][drop_node] + (i+1<n?travel_time[drop_node][order_list[i+1]] - travel_time[order_list[i]][order_list[i+1]]:0);
			if(insertion_cost>adj_insertion_cost){
				insertion_cost = adj_insertion_cost;
				best_pickup_loc = best_drop_loc = i; 
			}
			
			// pickup_cost = cost increment if we insert pickup Node just after i-th node.
			int pickup_cost = travel_time[order_list[i]][pickup_node] + (i+1<n?travel_time[pickup_node][order_list[i+1]] - travel_time[order_list[i]][order_list[i+1]]:0);
			//drop_cost = cost increment if we insert drop Node just after j-th node.
			int drop_cost = travel_time[order_list[i]][drop_node] + (i+1<n?travel_time[drop_node][order_list[i+1]] - travel_time[order_list[i]][order_list[i+1]]:0);
			
			if(insertion_cost > best_pickup_cost + drop_cost){
				insertion_cost = best_pickup_cost + drop_cost;
				best_drop_loc = i; 
				best_pickup_loc = best_pickup_loc_so_far;
			}
			if(best_pickup_cost > pickup_cost){
				best_pickup_cost = pickup_cost;
				best_pickup_loc_so_far = i;
			}
		}

		if(insert_in_list){
			order_list.insert(order_list.begin()+best_pickup_loc+1, pickup_node);
			order_list.insert(order_list.begin()+best_drop_loc+2, drop_node);
			curr_cost+=insertion_cost;
		}
		return insertion_cost;
	}

	void remove_order(int order){
		int removal_cost=0;
		int n=order_list.size();
		for(int i=2;i<order_list.size();i++){
			if(order_list[i] == 2*order){
				removal_cost += travel_time[order_list[i-1]][order_list[i]] + (i+1<n?travel_time[order_list[i]][order_list[i+1]] - travel_time[order_list[i-1]][order_list[i+1]]:0);
				order_list.erase(order_list.begin()+i);
			}
			if(order_list[i] == 2*order+1){
				removal_cost += travel_time[order_list[i-1]][order_list[i]] + (i+1<n?travel_time[order_list[i]][order_list[i+1]] - travel_time[order_list[i-1]][order_list[i+1]]:0);
				order_list.erase(order_list.begin()+i);
				break;
			}
		}

		curr_cost-=removal_cost;
	}


};

struct MASTER{
	int MAX_COST;  // maximum cost over all the costs of ROBOTS 
	vector<ROBOT> bots; // list of all ROBOT objects.
	int CURR_TIME;  // current time
	int Q_DOT;  // units of charge consumed per unit of time
	vector<int> deadlines; // deadliness for each order used for tardiness objective.
	vector<int> weights; // higher weights -> higher priority for an order.
	map <string, pair<int,vector<int>> > dp; // DP State
	int MAX_CHARGE;
	int THRESHOLD_CHARGE;
	map<int,pair<int,int>>movable_positions;
	
	MASTER(){
		MAX_COST=0;
		bots.clear();
		CURR_TIME=0;
	    Q_DOT = 10;
	    MAX_CHARGE=100;
	    THRESHOLD_CHARGE=30;
	    deadlines.clear();
		weights.clear();
		dp.clear();
	}

	// Cost that increases with time and rate depends on the order weights and deadlines.
	int delivery_cost(int order, int curr_time){
		return 0;
		// return weights[order]*max(0, curr_time-deadlines[order]) + weights[order]*max(0, curr_time-deadlines[order]/2);
	}	

	// HELPER function for conversion of a state to string.
	string convert_to_string(int a, int b, int c, int d){
		return to_string(a)+"#"+to_string(b)+"#"+to_string(c)+"#"+to_string(d);
	}

	// Returns the closest state : if found in the search space
	string find_closest_state(int mask_pickups, int mask_drops, int curr_order, int curr_time, int curr_charge){
		
		int time_tolerance = 5;

		for(int time=max(0,curr_time-time_tolerance);time<curr_time+time_tolerance;time++){
			string cl_state = to_string(mask_pickups)+"#"+to_string(mask_drops)+"#"+to_string(curr_order)+"#"+to_string(time);
			if(dp.find(cl_state)!=dp.end()){
				return cl_state;
			}
		}

		return "?";
	}

	////////////---- Local Search Part: ----///////////////
	
	void perform_order_swaps(int bot_key){


		for(int i=2;i<bots[bot_key].order_list.size();i++){
			vector<int> prev_order_list1 = bots[bot_key].order_list;
			
			int order1 = bots[bot_key].order_list[i];
			if(order1%2==1)continue;
			order1/=2;

			int best_new_state_cost=inf, best_bot2_key=-1, best_order2=-1;

			int prev_bot1_cost = bots[bot_key].curr_cost;
			bots[bot_key].remove_order(order1);

			for(auto bot2:bots){
				if(bots[bot_key].bot_key == bot2.bot_key)continue;
				vector<int> prev_order_list2 = bot2.order_list;

				int prev_state_cost = max(prev_bot1_cost, bot2.curr_cost);
				
				for(int j=2;j<bot2.order_list.size();j++){
					int order2 = bot2.order_list[j];
					if(order2%2==1)continue;
					order2/=2;	
					///////////////////	
					tot_swaps++;
					///////////////////
		
					bot2.remove_order(order2);
					int tardiness_cost=weights[order2]*max(0, CURR_TIME-deadlines[order2]);
					int new_state_cost = max(bots[bot_key].pseudo_insert(order2, 0)+tardiness_cost , bot2.pseudo_insert(order1, 0)+tardiness_cost);	
					bot2.order_list = prev_order_list2;

					if(best_new_state_cost > new_state_cost){
						best_new_state_cost = new_state_cost;
						best_bot2_key = bot2.bot_key;
						best_order2 = order2;
					}			
				}
			}

			if(best_bot2_key==-1){bots[bot_key].order_list = prev_order_list1; bots[bot_key].curr_cost = prev_bot1_cost; continue;}

			bots[best_bot2_key].remove_order(best_order2);
			bots[bot_key].pseudo_insert(best_order2, 1);
			bots[best_bot2_key].pseudo_insert(order1, 1);	
		}
	
	}

	void perform_order_relocations(int bot_key){
		for(int i=2;i<bots[bot_key].order_list.size();i++){
			vector<int> prev_order_list = bots[bot_key].order_list;
			
			int order = bots[bot_key].order_list[i];
			if(order%2==1)continue;
			order/=2;

			int best_new_state_cost=inf, best_bot2_key=-1, best_order2=-1;
			int prev_state_cost = bots[bot_key].curr_cost ;
			bots[bot_key].remove_order(order);

			for(auto bot2:bots){
				if(bot_key == bot2.bot_key)continue;

				prev_state_cost = max(prev_state_cost, bot2.curr_cost);
				int tardiness_cost=weights[order]*max(0, CURR_TIME-deadlines[order]);
				int new_state_cost = max(bots[bot_key].curr_cost , bot2.curr_cost+bot2.pseudo_insert(order, 0)+tardiness_cost);	

				if(best_new_state_cost > new_state_cost){
					best_new_state_cost = new_state_cost;
					best_bot2_key = bot2.bot_key;
				}			
			}

			if(best_bot2_key==-1){bots[bot_key].order_list = prev_order_list; bots[bot_key].curr_cost = prev_state_cost; continue;}
			
			i--;
			bots[best_bot2_key].pseudo_insert(order, true);	
		}
		tot_r++;
	}
	//////////-----LOCAL SEARCH ENDS-----///////////


	// For alloting new order to a BOT.
	void allot(int order){
		int alloted_bot_key = -1, mn_cost = inf;
		for(int i=0;i<bots.size();i++){
			int new_cost = bots[i].pseudo_insert(order, false);
			if(mn_cost > new_cost){
				mn_cost = new_cost;
				alloted_bot_key = i;
			}
		}
		assert(alloted_bot_key!=-1);
		bots[alloted_bot_key].order_list.push_back(2*order);bots[alloted_bot_key].order_list.push_back(2*order+1);
		tsp_update(alloted_bot_key);

		perform_order_relocations(alloted_bot_key);
		perform_order_swaps(alloted_bot_key);

		MAX_COST = max(MAX_COST, bots[alloted_bot_key].curr_cost);
	}

	void tsp_update(int bot_key){
		 dp.clear();
		
		// sort(bots[bot_key].order_list.begin()+2, bots[bot_key].order_list.end());
	
		tie(bots[bot_key].curr_cost, bots[bot_key].order_list) = tsp(bot_key, bots[bot_key].mask_pickups, bots[bot_key].mask_drops, 2*bot_key+1, CURR_TIME); // 1 -> 1st drop node = BOT
		
		bots[bot_key].order_list.push_back(2*bot_key+1),bots[bot_key].order_list.push_back(2*bot_key);
		reverse(bots[bot_key].order_list.begin(), bots[bot_key].order_list.end());
	}

	pair< int,vector<int> > tsp(int bot_key, int mask_pickups, int mask_drops, int curr_order, int curr_time){		
		int n=bots[bot_key].order_list.size()/2;
		
		int pickups = __builtin_popcount(mask_pickups);
		int drops = __builtin_popcount(mask_drops);
		int load = pickups - drops;
		assert((load>=0 && load<=2));
		if(drops == n){
			assert(pickups == n);
			return make_pair(0, vector<int> ());
		}

		string state = convert_to_string(mask_pickups,  mask_drops,  curr_order,  curr_time);
		// string closest_state = find_closest_state(mask_pickups, mask_drops, curr_order, curr_time, curr_charge);
		if(dp.find(state)!=dp.end()){
			return	dp[state];
		}

		int ans = inf;
		vector<int> opt_path;

		// going for another pickup if load<=1;
		if(load<=1){
			for(int next_order:bots[bot_key].order_list){
				if(next_order<0 || next_order%2==1)continue;  // not a pickup node
				int loc = (next_order)/2;
				
				int next_time = curr_time + travel_time[curr_order][next_order];
				if((mask_pickups&(1<<loc))==0){
					pair<int,vector<int>> val = tsp(bot_key, (mask_pickups|(1<<loc)), mask_drops, next_order, next_time);	
					int newMin = travel_time[curr_order][next_order] + val.first;
					if(ans>newMin){
						ans=newMin;
						vector<int> dum = val.second;
						dum.push_back(next_order);
						opt_path  = dum;
					}
				}
			}
		}	

		// going for a drop if load>0 AND adding a tardiness cost for passing deadlines
		if(load>0){
			for(int next_order:bots[bot_key].order_list){
				if(next_order<0 || next_order%2==0)continue;  // not a drop node
				int loc = (next_order)/2;
				int next_time = curr_time+travel_time[curr_order][next_order];
				if((mask_drops&(1<<loc))==0 and (mask_pickups&(1<<loc))>0 ){
					pair<int,vector<int>> val = tsp(bot_key, mask_pickups, (mask_drops|(1<<loc)), next_order, next_time);
					int newMin = travel_time[curr_order][next_order] + val.first + delivery_cost(loc, next_time);
					if(ans>newMin){
						ans=newMin;
						vector<int> dum = val.second;
						dum.push_back(next_order);
						opt_path = dum;
					}
				}
			}
		}
		
		return dp[state] = make_pair(ans, opt_path);

	}


	// For alloting a new order to the optimal BOT.
	void allot_with_constraints(int order){
		int alloted_bot_key = -1, mn_cost = INT_MAX;
		for(int i=0;i<bots.size();i++){
			int new_cost = bots[i].pseudo_insert(order,false);
			if(new_cost < mn_cost){
				mn_cost = new_cost;
				alloted_bot_key = i;
			}
		}
		assert(alloted_bot_key!=-1);
		bots[alloted_bot_key].order_list_with_constraints.push_back(2*order);bots[alloted_bot_key].order_list_with_constraints.push_back(2*order+1);
		tsp_with_constraints_update(alloted_bot_key);
		MAX_COST = max(MAX_COST, bots[alloted_bot_key].curr_cost);
	}

	void tsp_with_constraints_update(int bot_key){
		dp.clear();
		sort(bots[bot_key].order_list_with_constraints.begin()+2, bots[bot_key].order_list_with_constraints.end()); // the first 2 nodes are associated with the BOT;
		tie(bots[bot_key].curr_cost, bots[bot_key].order_list_with_constraints) = tsp_with_constraints(bot_key, bots[bot_key].mask_pickups, bots[bot_key].mask_drops, 2*bot_key+1, CURR_TIME, bots[bot_key].curr_charge); // 1 -> 1st drop node = BOT
		bots[bot_key].order_list_with_constraints.push_back(2*bot_key+1),bots[bot_key].order_list_with_constraints.push_back(2*bot_key);
		reverse(bots[bot_key].order_list_with_constraints.begin(), bots[bot_key].order_list_with_constraints.end());
	}

	pair< int,vector<int> > tsp_with_constraints(int bot_key, int mask_pickups, int mask_drops, int curr_order, int curr_time, int curr_charge){
		int n=0;
		for(int x:bots[bot_key].order_list){
			if(x!=-1)n++;
		}
		n/=2;

		int pickups = __builtin_popcount(mask_pickups);
		int drops = __builtin_popcount(mask_drops);
		int load = pickups - drops;
		assert((load>=0 && load<=2));
		if(drops == n){
			assert(pickups == n);
			return make_pair(0, vector<int> ());
		}

		string closest_state = find_closest_state(mask_pickups, mask_drops, curr_order, curr_time, curr_charge);
		if(closest_state!="?"){
			return	dp[closest_state];
		}
		string state = convert_to_string(mask_pickups,  mask_drops,  curr_order,  curr_time);

		int ans = inf;
		vector<int> opt_path;

		// going for another pickup if load<=1;
		if(load<=1){
			for(int next_order:bots[bot_key].order_list){
				if(next_order<0 || next_order%2==1)continue;  // not a pickup node
				int loc = (next_order)/2;
				int next_charge = curr_charge - Q_DOT*travel_time[curr_order][next_order];
				int next_time = curr_time + travel_time[curr_order][next_order];
				if((mask_pickups&(1<<loc))==0 and next_charge>0 ){
					pair<int,vector<int>> val = tsp_with_constraints(bot_key, (mask_pickups|(1<<loc)), mask_drops, next_order, next_time, next_charge);
					
					if(movable_positions[curr_order].first<=val.second.size() and movable_positions[curr_order].second>=val.second.size())
					{
					int newMin = travel_time[curr_order][next_order] + val.first;
					if(ans>newMin){
						ans=newMin;
						vector<int> dum = val.second;
						dum.push_back(next_order);
						opt_path  = dum;
					}
					}
				}
			}
		}	

		// going for a drop if load>0 AND adding a tardiness cost for passing deadlines
		if(load>0){
			for(int next_order:bots[bot_key].order_list){
				if(next_order<0 || next_order%2==0)continue;  // not a drop node
				int loc = (next_order)/2;
				int next_charge = curr_charge - Q_DOT*travel_time[curr_order][next_order];
				int next_time = curr_time+travel_time[curr_order][next_order];
				if((mask_drops&(1<<loc))==0 and (mask_pickups&(1<<loc))>0  and next_charge>0){
					pair<int,vector<int>> val = tsp_with_constraints(bot_key, mask_pickups, (mask_drops|(1<<loc)), next_order, next_time, next_charge);
					
					if(movable_positions[curr_order].first<=val.second.size() and movable_positions[curr_order].second>=val.second.size())
					{
					int newMin = travel_time[curr_order][next_order] + val.first + delivery_cost(loc, next_time);
					if(ans>newMin){
						ans=newMin;
						vector<int> dum = val.second;
						dum.push_back(next_order);
						opt_path = dum;
					}
					}
				}
			}
		}


		return dp[state] = make_pair(ans, opt_path);
	}
/////////<---Charging--->///////////////
int nearest_charging_node(int prev_node,int next_node)
{
	return 100;
}
	
	void charging(int bot_key,int constraint_variable)
{
	set<int>pickup_order;
	int current_load=0;
	if(constraint_variable==0)
	{
	if(bots[bot_key].order_list[2]%2==0)
	{
	bots[bot_key].order_list_with_charging.push_back(bots[bot_key].order_list[2]);
	pickup_order.insert(bots[bot_key].order_list[2]);
	current_load++;
	}
	for(int i=3;i<bots[bot_key].order_list.size();++i)
	{
		int prev_node=bots[bot_key].order_list[i-1];
		int curr_node=bots[bot_key].order_list[i];
		
		if(curr_node%2==0)
		{
			current_load++;
			pickup_order.insert(curr_node);
		}
		if(pickup_order.find(curr_node-1)!=pickup_order.end())
		{
			current_load--;
			pickup_order.erase(curr_node-1);
		}
		if(current_load==0)
		{
		if((MAX_CHARGE-(Q_DOT*(travel_time[prev_node][curr_node])))<=THRESHOLD_CHARGE)
		{
			int charging_station=nearest_charging_node(prev_node,curr_node);
			bots[bot_key].order_list_with_charging.push_back(curr_node);
			bots[bot_key].order_list_with_charging.push_back(charging_station);
		}
		else
		{
			bots[bot_key].order_list_with_charging.push_back(curr_node);
		}
		}
		else if(current_load!=0)
		{
			bots[bot_key].order_list_with_charging.push_back(curr_node);
		}
	}
	pickup_order.clear();
	}
	else if(constraint_variable==1)
	{
	if(bots[bot_key].order_list[2]%2==0)
	{
	bots[bot_key].order_list_with_constraint_charging.push_back(bots[bot_key].order_list[2]);
	pickup_order.insert(bots[bot_key].order_list[2]);
	current_load++;
	}
	for(int i=3;i<bots[bot_key].order_list.size();++i)
	{
		int prev_node=bots[bot_key].order_list[i-1];
		int curr_node=bots[bot_key].order_list[i];
		
		if(curr_node%2==0)
		{
			current_load++;
			pickup_order.insert(curr_node);
		}
		if(pickup_order.find(curr_node-1)!=pickup_order.end())
		{
			current_load--;
			pickup_order.erase(curr_node-1);
		}
		if(current_load==0)
		{
		if((MAX_CHARGE-(Q_DOT*(travel_time[prev_node][curr_node])))<=THRESHOLD_CHARGE)
		{
			int charging_station=nearest_charging_node(prev_node,curr_node);
			bots[bot_key].order_list_with_constraint_charging.push_back(curr_node);
			bots[bot_key].order_list_with_constraint_charging.push_back(charging_station);
		}
		else
		{
			bots[bot_key].order_list_with_constraint_charging.push_back(curr_node);
		}
		}
		else if(current_load!=0)
		{
			bots[bot_key].order_list_with_charging.push_back(curr_node);
		}
	}
	pickup_order.clear();		
	}
}
};

// Change it for multiple Bots.  -> DONE
// implement closest_node function.  -> DONE
// Accomodate charging in pseudo insertion cost function -> Working on it.
// constraints for priority order alterations. -> R.P

void initialize_travel_time_matrix(int sz){
	for(int i=0;i<sz;i++){
		vector<int> rand;
		for(int j=0;j<sz;j++){
			rand.push_back(uniform_int_distribution<int>(1,10)(rng));
		}
		travel_time.push_back(rand);
	}
}


void print_schedule(MASTER master,int with_constraints)
{
	if(with_constraints==0)
	{
		cout<<"Updated Lists :\n";
		for(int i=0;i<master.bots.size();i++){
			cout<<"BOT "<<i<<" | "<<"COST "<<master.bots[i].curr_cost<<" -> ";
			for(int j=2;j<master.bots[i].order_list.size();j++){
				int x = master.bots[i].order_list[j];
				if(x==100)cout<<" Charge "; 
				else {cout<<(x%2?"Drop":"Pick")<<x/2<<" ";}
			}cout<<endl;
		}	
		cout<<"Total Swaps: "<<tot_swaps<<endl;
		cout<<"Total Relocations"<<tot_r<<endl;
	}
	if(with_constraints==1)
	{
		cout<<"Updated Lists :\n";
		for(int i=0;i<master.bots.size();i++){
			cout<<"BOT "<<i<<" | "<<"COST "<<master.bots[i].curr_cost<<" -> ";
			for(int j=2;j<master.bots[i].order_list_with_constraints.size();j++){
				int x = master.bots[i].order_list_with_constraints[j];
				if(x==100) cout<<" Charge ";
				else {cout<<(x%2?"Drop":"Pick")<<x/2<<" ";}
			}cout<<endl;
		}
		cout<<"Total Swaps: "<<tot_swaps<<endl;
		cout<<"Total Relocations"<<tot_r<<endl;	
	}
}
void scheduling_without_constraints(MASTER &master,int no_of_orders,int no_of_bots,int output_schedule)
{
		// Order Description
	for(int order_id=no_of_bots ; order_id<no_of_orders+no_of_bots ; order_id++){
		
		master.deadlines.push_back(0);
		master.weights.push_back(1);
		master.allot(order_id);
		if(output_schedule==1)
		{
			print_schedule(master,0);
		}
	}
}

void scheduling_with_constraints(MASTER &master,int no_of_orders,int no_of_bots)
{
	int range=0;
	scheduling_without_constraints(master,no_of_orders,no_of_bots,0);
	for(int i=0;i<master.bots.size();i++){
		int size_of_list=master.bots[i].order_list.size();
		for(int j=2;j<master.bots[i].order_list.size();j++){
			int order_list_value = master.bots[i].order_list[j];
			cout<<"Enter range of movable positions for  "<<(order_list_value%2?"Drop":"Pick")<<order_list_value/2<<" "<<" for bot "<<i<<"\n";
			cin>>range;
			master.movable_positions[order_list_value]={max(0,j-range),min(j+range,size_of_list)};
			}
		}
	for(int order_id=no_of_bots ; order_id<no_of_orders+no_of_bots ; order_id++){
		
		master.deadlines.push_back(0);
		master.weights.push_back(1);
		master.allot_with_constraints(order_id);
			print_schedule(master,1);
	}
}
signed main(){

	MASTER master;
	cout<<"No of BOTS:\n";
	int no_of_bots;
	cin>>no_of_bots;

	// BOT Description
	for(int bot_id=0;bot_id<no_of_bots;bot_id++){
		ROBOT robot(bot_id);
		master.bots.push_back(robot);
		master.deadlines.push_back(0);
		master.weights.push_back(1);
	}

	cout<<"No of Orders:\n";
	int no_of_orders;
	cin>>no_of_orders;

	// randomly fills the travel_time matrix with time from each node to another.
	initialize_travel_time_matrix(2*(no_of_orders + no_of_bots));

////<--Hard Constraints Routine-->////
	int constraint_variable=0;
	cout<<"Enter if constraints are available \n";
	cin>>constraint_variable;
	if(constraint_variable==0)
	{
	scheduling_without_constraints(master,no_of_orders,no_of_bots,1);
	}
	else if(constraint_variable==1)
	{
	scheduling_with_constraints(master,no_of_orders,no_of_bots);
	}
	
	
	cout<<"After charging\n";
	if(constraint_variable==0)
	{  for(int i=0;i<master.bots.size();++i)
        {
        	master.charging(i,constraint_variable);
        }

	   	for(int i=0;i<master.bots.size();i++){
			cout<<"BOT "<<i<<" | "<<"COST "<<master.bots[i].curr_cost<<" -> ";
			for(int j=0;j<master.bots[i].order_list_with_charging.size();j++){
				int x = master.bots[i].order_list_with_charging[j];
				if(x==100)cout<<" Charge ";
				else {cout<<(x%2?"Drop":"Pick")<<x/2<<" ";}
			}cout<<endl;
		}
	}
	else if(constraint_variable==1)
	{
	  for(int i=0;i<master.bots.size();++i)
        {
        	master.charging(i,constraint_variable);
        }

	   	for(int i=0;i<master.bots.size();i++){
			cout<<"BOT "<<i<<" | "<<"COST "<<master.bots[i].curr_cost<<" -> ";
			for(int j=0;j<master.bots[i].order_list_with_constraint_charging.size();j++){
				int x = master.bots[i].order_list_with_constraint_charging[j];
				if(x==100)cout<<" Charge ";
				else {cout<<(x%2?"Drop":"Pick")<<x/2<<" ";}
			}cout<<endl;
		}	
	}
    return 0;
}